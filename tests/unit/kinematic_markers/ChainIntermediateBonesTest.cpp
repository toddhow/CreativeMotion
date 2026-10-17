// A4c-1: IK chains whose bones have other bones between them (docs/tasks/0008-a4c1-chain-
// intermediate-bones.md). The scenes use the humanoid of NodedHumanoid.hpp, with non-identity
// intermediate nodes between upper arm and forearm and between shin and foot.
//
// alignPair reports world rotations, not a pose, so the tests apply them themselves, following rule
// 3 with their own forward kinematics: set the root bone's world rotation (rootRotation * old),
// recompute, then set the mid bone's (midRotation * its old world rotation, from before the chain
// was solved) with the local rotation taken against the mid bone's actual parent in the recomputed
// pose, recompute. Rule 4 is then checked on that pose.

#include "AlignmentFlatten.hpp"
#include "AlignmentGoldens.hpp"
#include "AlignmentOracle.hpp"
#include "AlignmentScenes.hpp"
#include "NodedHumanoid.hpp"
#include "SyntheticHumanoid.hpp"
#include "TestSupport.hpp"
#include "opas/markers/Alignment.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {

using opas::markers::AlignmentResult;
using opas::markers::IkChain;
using opas::markers::LimbCorrection;
using opas::math::conjugate;
using opas::math::distance;
using opas::math::normalized;
using opas::math::Quat;
using opas::math::Transform;
using opas::math::Vec3;
using opas::test::alignScene;
using opas::test::bonePosition;
using opas::test::Humanoid;
using opas::test::makeMarkerAt;
using opas::test::makeNodedHumanoid;
using opas::test::Scene;
using opas::test::SceneActor;
using opas::test::shiftedScene;
using opas::test::standingAt;

constexpr double kTol = 1e-9;

// ---- The scene
// -----------------------------------------------------------------------------------------

SceneActor nodedActor(double scale, const Transform& root)
{
    SceneActor a;
    a.body = makeNodedHumanoid(scale);
    a.pose = a.body.restPose;
    a.root = root;
    return a;
}

// Two noded humanoids face to face 0.6 m apart; actor 0's right hand and right foot are asked to
// reach points that need the roots to move and the limbs to bend.
Scene nodedScene(double scale0 = 1.0, double scale1 = 1.0)
{
    Scene s;
    s.actors[0] = nodedActor(scale0, standingAt(0.0, 0.0, false));
    s.actors[1] = nodedActor(scale1, standingAt(0.0, 0.6, true));
    // Hand: 0.15 m up and 0.2 m forward of where it hangs; foot: 0.2 m up and 0.15 m forward.
    const Vec3 hand = bonePosition(s.actors[0], "RightHand") + Vec3{0.05, 0.2, 0.15};
    const Vec3 foot = bonePosition(s.actors[0], "RightFoot") + Vec3{0.05, 0.15, 0.2};
    s.markers.markers.push_back(makeMarkerAt(s, "hand", 0, "RightHand", 1, "Torso", hand));
    s.markers.markers.push_back(makeMarkerAt(s, "foot", 0, "RightFoot", 1, "Torso", foot));
    return s;
}

std::size_t index(const SceneActor& a, const std::string& name)
{
    return opas::test::boneIndex(a, name);
}

// ---- Rule 3 applied by the test
// --------------------------------------------------------------------------

struct AppliedPose {
    std::vector<Transform> before;
    std::vector<Transform> after;
};

AppliedPose applyPerRuleThree(const SceneActor& actor, const IkChain& chain,
                              const LimbCorrection& limb)
{
    const auto& skeleton = actor.body.skeleton;
    std::vector<Transform> local = actor.pose;
    AppliedPose out;
    out.before = opas::test::worldPose(skeleton, local, actor.root);

    const auto parentRotation = [&](const std::vector<Transform>& world, std::size_t i) {
        const int p = skeleton.bones[i].parent;
        return p < 0 ? actor.root.rotation : world[static_cast<std::size_t>(p)].rotation;
    };
    const std::size_t root = index(actor, chain.rootBone);
    const std::size_t mid = index(actor, chain.midBone);

    // 1. The root bone.
    const Quat newRoot = normalized(limb.rootBoneDelta * out.before[root].rotation);
    local[root].rotation = normalized(conjugate(parentRotation(out.before, root)) * newRoot);
    const std::vector<Transform> afterRoot = opas::test::worldPose(skeleton, local, actor.root);

    // 2. The mid bone: its old world rotation is the one from before the chain was solved; its
    // parent is whatever its actual parent is now.
    const Quat newMid = normalized(limb.midBoneDelta * out.before[mid].rotation);
    local[mid].rotation = normalized(conjugate(parentRotation(afterRoot, mid)) * newMid);
    out.after = opas::test::worldPose(skeleton, local, actor.root);
    return out;
}

const LimbCorrection* limbOf(const AlignmentResult& r, const std::string& chain)
{
    for (const LimbCorrection& l : r.limbs) {
        if (l.chain == chain) {
            return &l;
        }
    }
    return nullptr;
}

void expectChainSolved(const Scene& scene, const AlignmentResult& r, const std::string& chainName,
                       const std::string& markerId)
{
    SCOPED_TRACE(chainName);
    const LimbCorrection* limb = limbOf(r, chainName);
    ASSERT_NE(limb, nullptr);
    EXPECT_TRUE(limb->reached);
    EXPECT_EQ(limb->markerId, markerId);

    const Scene shifted = shiftedScene(scene, r);
    const SceneActor& actor = shifted.actors[static_cast<std::size_t>(limb->actor)];
    const IkChain* chain = opas::test::findChain(actor, chainName);
    ASSERT_NE(chain, nullptr);
    const AppliedPose pose = applyPerRuleThree(actor, *chain, *limb);

    const opas::markers::InteractionMarker* marker = opas::test::findMarker(scene, markerId);
    ASSERT_NE(marker, nullptr);
    const Vec3 target = opas::test::markerTargetPoint(shifted, *marker);
    const std::size_t end = index(actor, chain->endBone);

    // Rule 4: the end bone is at the solved end (the target, as the chain reaches it)...
    EXPECT_LE(distance(pose.after[end].translation, target), kTol);
    // ...it was not there before (the scene needs the IK)...
    EXPECT_GT(distance(pose.before[end].translation, target), 0.05);
    // ...the positions agree with the vector form the A2 oracle uses...
    const opas::test::SolvedLimb solved = opas::test::solvedLimb(shifted, *limb);
    EXPECT_LE(distance(pose.after[index(actor, chain->midBone)].translation, solved.mid), kTol);
    EXPECT_LE(distance(pose.after[end].translation, solved.end), kTol);
    // ...and every bone's distance to its parent is unchanged.
    for (std::size_t i = 0; i < actor.body.skeleton.bones.size(); ++i) {
        const int parent = actor.body.skeleton.bones[i].parent;
        if (parent < 0) {
            continue;
        }
        const auto p = static_cast<std::size_t>(parent);
        EXPECT_NEAR(distance(pose.after[i].translation, pose.after[p].translation),
                    distance(pose.before[i].translation, pose.before[p].translation), kTol)
            << actor.body.skeleton.bones[i].name;
    }
}

// ---- Acceptance: chains through intermediate bones
// ------------------------------------------------------------

TEST(ChainIntermediates, TheNodedHumanoidReallyHasNonIdentityNodesInTheChains)
{
    const Humanoid h = makeNodedHumanoid(1.0);
    const auto find = [&](const std::string& name) {
        return opas::rig::findBone(h.skeleton, name);
    };
    ASSERT_TRUE(find("RightForearmRoll").has_value());
    ASSERT_TRUE(find("RightShinNodeA").has_value());
    ASSERT_TRUE(find("RightShinNodeB").has_value());
    for (const char* name : {"RightForearmRoll", "RightShinNodeA", "RightShinNodeB",
                             "LeftForearmRoll", "LeftShinNodeA", "LeftShinNodeB"}) {
        SCOPED_TRACE(name);
        const Transform& t = h.skeleton.bones[find(name).value_or(0)].localRest;
        EXPECT_NE(t.rotation, Quat{});
        EXPECT_GT(opas::math::length(t.translation), 0.0);
    }
    // The arm's mid bone is two levels below its root bone; the leg's end bone three below its mid.
    EXPECT_EQ(h.skeleton.bones[find("RightForearm").value_or(0)].parent,
              static_cast<int>(find("RightForearmRoll").value_or(0)));
    EXPECT_EQ(h.skeleton.bones[find("RightForearmRoll").value_or(0)].parent,
              static_cast<int>(find("RightUpperArm").value_or(0)));
    EXPECT_EQ(h.skeleton.bones[find("RightFoot").value_or(0)].parent,
              static_cast<int>(find("RightShinNodeB").value_or(0)));
    EXPECT_EQ(h.skeleton.bones[find("RightShinNodeB").value_or(0)].parent,
              static_cast<int>(find("RightShinNodeA").value_or(0)));
    EXPECT_EQ(h.skeleton.bones[find("RightShinNodeA").value_or(0)].parent,
              static_cast<int>(find("RightShin").value_or(0)));
    EXPECT_EQ(opas::rig::validateSkeleton(h.skeleton), "");
}

TEST(ChainIntermediates, AFootMarkerAndAHandMarkerAreAlignedThroughChainsThatSkipTheNodes)
{
    for (const auto& [s0, s1] :
         std::array<std::pair<double, double>, 3>{{{1.0, 1.0}, {0.8, 1.25}, {1.25, 0.8}}}) {
        SCOPED_TRACE(std::to_string(s0) + " / " + std::to_string(s1));
        const Scene scene = nodedScene(s0, s1);
        const AlignmentResult r = alignScene(scene);
        ASSERT_TRUE(r.valid) << r.error;
        EXPECT_FALSE(r.rootShiftClamped);
        ASSERT_EQ(r.limbs.size(), 2U);
        EXPECT_EQ(r.limbs[0].chain, "RightArm"); // mapping order: arms before legs
        EXPECT_EQ(r.limbs[1].chain, "RightLeg");
        // Markers met as in A2 (the end bone is at the target to rounding).
        ASSERT_EQ(r.markers.size(), 2U);
        for (const auto& m : r.markers) {
            EXPECT_TRUE(m.met) << m.id << " " << m.distanceM;
            EXPECT_LE(m.distanceM, kTol) << m.id;
        }
        expectChainSolved(scene, r, "RightArm", "hand");
        expectChainSolved(scene, r, "RightLeg", "foot");
    }
}

TEST(ChainIntermediates, AHandAloneAndAFootAloneAreEachSolved)
{
    for (const char* only : {"hand", "foot"}) {
        SCOPED_TRACE(only);
        Scene scene = nodedScene();
        std::erase_if(scene.markers.markers,
                      [&](const opas::markers::InteractionMarker& m) { return m.id != only; });
        const AlignmentResult r = alignScene(scene);
        ASSERT_TRUE(r.valid) << r.error;
        ASSERT_EQ(r.limbs.size(), 1U);
        ASSERT_EQ(r.markers.size(), 1U);
        EXPECT_TRUE(r.markers[0].met);
        EXPECT_LE(r.markers[0].distanceM, kTol);
        expectChainSolved(scene, r, only == std::string("hand") ? "RightArm" : "RightLeg", only);
    }
}

TEST(ChainIntermediates, TheLeftSideWorksToo)
{
    Scene scene = nodedScene();
    // Mirror the targets onto the left limbs.
    const Vec3 hand = bonePosition(scene.actors[0], "LeftHand") + Vec3{-0.05, 0.2, 0.15};
    const Vec3 foot = bonePosition(scene.actors[0], "LeftFoot") + Vec3{-0.05, 0.15, 0.2};
    scene.markers.markers.clear();
    scene.markers.markers.push_back(makeMarkerAt(scene, "hand", 0, "LeftHand", 1, "Torso", hand));
    scene.markers.markers.push_back(makeMarkerAt(scene, "foot", 0, "LeftFoot", 1, "Torso", foot));
    const AlignmentResult r = alignScene(scene);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 2U);
    expectChainSolved(scene, r, "LeftArm", "hand");
    expectChainSolved(scene, r, "LeftLeg", "foot");
}

TEST(ChainIntermediates, AChainMayStartHigherUpAndStillBeAccepted)
{
    // Root = Chest, mid = Forearm (three levels below), end = Hand: any depth is allowed.
    Scene scene = nodedScene();
    for (SceneActor& actor : scene.actors) {
        actor.body.mapping.ikChains[0].rootBone = "Chest";
    }
    std::erase_if(scene.markers.markers,
                  [](const opas::markers::InteractionMarker& m) { return m.id != "hand"; });
    const AlignmentResult r = alignScene(scene);
    EXPECT_TRUE(r.valid) << r.error;
}

TEST(ChainIntermediates, DirectParentChildChainsInTheNodedSkeletonStayValid)
{
    // Upper arm -> roll -> forearm: each a direct child of the one before.
    Scene scene = nodedScene();
    IkChain& chain = scene.actors[0].body.mapping.ikChains[0];
    chain.rootBone = "RightUpperArm";
    chain.midBone = "RightForearmRoll";
    chain.endBone = "RightForearm";
    scene.markers.markers.clear();
    const AlignmentResult r = alignScene(scene);
    EXPECT_TRUE(r.valid) << r.error;
}

// ---- Rejection: the chain's bones must descend from one another
// ------------------------------------------------

void setChain(Scene& scene, int actor, std::size_t chainIndex, const std::string& root,
              const std::string& mid, const std::string& end)
{
    IkChain& chain =
        scene.actors[static_cast<std::size_t>(actor)].body.mapping.ikChains[chainIndex];
    chain.rootBone = root;
    chain.midBone = mid;
    chain.endBone = end;
}

void expectRejectedNaming(const Scene& scene, const std::string& chainName)
{
    const AlignmentResult r = alignScene(scene);
    EXPECT_FALSE(r.valid);
    EXPECT_FALSE(r.error.empty());
    EXPECT_NE(r.error.find(chainName), std::string::npos) << r.error;
}

TEST(ChainIntermediatesRejected, AMidBoneThatIsNotBelowTheRootBone)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        // A sibling branch: the left forearm is not below the right upper arm.
        Scene sibling = nodedScene();
        setChain(sibling, actor, 0, "RightUpperArm", "LeftForearm", "LeftHand");
        expectRejectedNaming(sibling, "RightArm");

        // Reversed: the root bone is below the mid bone.
        Scene reversed = nodedScene();
        setChain(reversed, actor, 0, "RightForearm", "RightUpperArm", "RightHand");
        expectRejectedNaming(reversed, "RightArm");

        // The mid bone is an ancestor of the root bone.
        Scene above = nodedScene();
        setChain(above, actor, 0, "RightUpperArm", "Chest", "RightHand");
        expectRejectedNaming(above, "RightArm");
    }
}

TEST(ChainIntermediatesRejected, AnEndBoneThatIsNotBelowTheMidBone)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        // Another branch altogether.
        Scene sibling = nodedScene();
        setChain(sibling, actor, 0, "RightUpperArm", "RightForearm", "LeftHand");
        expectRejectedNaming(sibling, "RightArm");

        // Reversed: the end bone is the root bone (above the mid bone).
        Scene reversed = nodedScene();
        setChain(reversed, actor, 0, "RightUpperArm", "RightForearm", "RightUpperArm");
        expectRejectedNaming(reversed, "RightArm");

        // The end bone is an ancestor of the root bone.
        Scene above = nodedScene();
        setChain(above, actor, 0, "RightUpperArm", "RightForearm", "Chest");
        expectRejectedNaming(above, "RightArm");

        // A leg chain whose end bone is a node of another limb.
        Scene otherLeg = nodedScene();
        setChain(otherLeg, actor, 2, "RightThigh", "RightShin", "LeftShinNodeA");
        expectRejectedNaming(otherLeg, "RightLeg");
    }
}

TEST(ChainIntermediatesRejected, TheChainIsNamedEvenWhenNoMarkerUsesIt)
{
    Scene scene = nodedScene();
    setChain(scene, 0, 3, "LeftThigh", "RightShin",
             "LeftFoot"); // the left leg, unused by any marker
    expectRejectedNaming(scene, "LeftLeg");
}

TEST(ChainIntermediatesRejected, TheMessageNamesTheChainThatBreaksTheRule)
{
    Scene scene = nodedScene();
    setChain(scene, 1, 1, "LeftUpperArm", "RightForearm", "RightHand"); // actor 1's left arm
    const AlignmentResult r = alignScene(scene);
    ASSERT_FALSE(r.valid);
    EXPECT_NE(r.error.find("LeftArm"), std::string::npos) << r.error;
}

// ---- Direct chains: bit-identical to before
// -----------------------------------------------------------------------

void expectGolden(const char* name, const Scene& scene, std::span<const std::uint64_t> golden)
{
    SCOPED_TRACE(name);
    const std::vector<std::uint64_t> now = opas::test::flatten(alignScene(scene));
    ASSERT_EQ(now.size(), golden.size());
    for (std::size_t i = 0; i < now.size(); ++i) {
        EXPECT_EQ(now[i], golden[i]) << "word " << i;
    }
}

// The golden words were recorded with alignPair as it was before A4c-1 (git 737c298): every chain
// in these scenes joins a bone to its direct child, so the results must not change at all.
TEST(ChainDirectUnchanged, TheA2ScenesGiveBitIdenticalResultsToBeforeTheChange)
{
    namespace g = opas::test::golden;
    expectGolden("hug 0.8/1.25", opas::test::makeHugScene(0.8, 1.25), g::kHug08_125);
    expectGolden("hug 1.25/0.8", opas::test::makeHugScene(1.25, 0.8), g::kHug125_08);
    expectGolden("hug 1/1", opas::test::makeHugScene(1.0, 1.0), g::kHug1_1);
    expectGolden("handshake 0.8/1.25", opas::test::makeHandshakeScene(0.8, 1.25, 0),
                 g::kShake08_125);
    expectGolden("handshake 1.25/0.8 hand of actor 1", opas::test::makeHandshakeScene(1.25, 0.8, 1),
                 g::kShake125_08Src1);
    expectGolden("shoulder 0.8/1.25", opas::test::makeShoulderScene(0.8, 1.25), g::kShoulder08_125);
    expectGolden("shoulder 1.25/0.8", opas::test::makeShoulderScene(1.25, 0.8), g::kShoulder125_08);
    expectGolden("penetration", opas::test::makePenetrationScene(), g::kPenetration);
    Scene weighted = opas::test::makeShoulderScene(1.0, 1.25);
    weighted.actors[0].weight = 3.0;
    weighted.actors[1].weight = 1.0;
    expectGolden("shoulder weighted", weighted, g::kShoulderWeighted);
}

} // namespace
