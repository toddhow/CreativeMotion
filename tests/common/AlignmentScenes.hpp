#pragma once

// Synthetic two-actor scenes for the alignment tests (A2). Every scene is built from
// the humanoid in SyntheticHumanoid.hpp; marker offsets are computed here from the
// bodies' own geometry (so they are reachable), never typed in as magic numbers.
//
// How a scene is made ("clip authored for a different arrangement"):
//   1. Put the two actors in a reference arrangement F, facing each other.
//   2. Pick the contact points in the world at F (on the other actor's body surface).
//   3. Pose each driven arm so its hand is near its contact point, as an animation clip
//      would (the clip's hand is deliberately a little off the contact).
//   4. Write each marker's offset in the target region's frame at F.
//   5. Shift one actor's root, so the solver has real work to do.

#include "SyntheticHumanoid.hpp"
#include "opas/ik/TwoBoneIk.hpp"
#include "opas/markers/Alignment.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace opas::test {

inline constexpr double kScenePi = 3.14159265358979323846;

// One actor in a scene: the body, its local pose, and where it stands.
struct SceneActor {
    Humanoid body;
    std::vector<math::Transform> pose; // local, one per bone
    math::Transform root;              // model -> world
    double weight = 1.0;
};

struct Scene {
    SceneActor actors[2];
    markers::MarkerSet markers;
    double clipTimeS = 1.0;
};

// Root transform at (x, y) on the ground; faceMinusY turns the actor to face -Y.
inline math::Transform standingAt(double x, double y, bool faceMinusY)
{
    const math::Quat rotation =
        faceMinusY ? math::fromAxisAngle(math::Vec3{0.0, 0.0, 1.0}, kScenePi) : math::Quat{};
    return math::Transform{math::Vec3{x, y, 0.0}, rotation};
}

inline SceneActor makeSceneActor(double scale, const math::Transform& root)
{
    SceneActor a;
    a.body = makeHumanoid(scale);
    a.pose = a.body.restPose;
    a.root = root;
    return a;
}

// The alignment solver's view of a scene actor. References the scene: keep it alive.
inline markers::ActorInput inputOf(const SceneActor& a)
{
    return markers::ActorInput{a.body.skeleton, a.body.mapping, std::span(a.pose), a.root,
                               a.weight};
}

inline markers::AlignmentResult alignScene(const Scene& scene,
                                           const markers::AlignmentSettings& settings = {})
{
    return markers::alignPair(inputOf(scene.actors[0]), inputOf(scene.actors[1]), scene.markers,
                              scene.clipTimeS, settings);
}

// ---- Geometry helpers (all in the world, for the actor as currently placed) -------------

inline std::vector<math::Transform> worldOf(const SceneActor& a)
{
    return worldPose(a.body.skeleton, a.pose, a.root);
}

inline std::size_t boneIndex(const SceneActor& a, const std::string& name)
{
    return rig::findBone(a.body.skeleton, name).value_or(0);
}

inline const markers::CapsuleRule& capsuleRule(const SceneActor& a, const std::string& region)
{
    for (const markers::CapsuleRule& c : a.body.mapping.capsules) {
        if (c.region == region) {
            return c;
        }
    }
    return a.body.mapping.capsules.front(); // not reached for the regions the scenes use
}

inline math::Transform regionFrame(const SceneActor& a, const std::string& region)
{
    return worldOf(a)[boneIndex(a, capsuleRule(a, region).bone)];
}

// Point a fraction t (0..1) along a region's capsule axis.
inline math::Vec3 capsuleAxisPoint(const SceneActor& a, const std::string& region, double t)
{
    const std::vector<math::Transform> w = worldOf(a);
    const markers::CapsuleRule& c = capsuleRule(a, region);
    const math::Vec3 start = w[boneIndex(a, c.bone)].translation;
    const math::Vec3 end = w[boneIndex(a, c.endBone)].translation;
    return start + (end - start) * t;
}

inline double capsuleRadiusM(const SceneActor& a, const std::string& region)
{
    const std::vector<math::Transform> w = worldOf(a);
    const markers::CapsuleRule& c = capsuleRule(a, region);
    return c.radiusRatio * math::length(w[boneIndex(a, c.endBone)].translation -
                                        w[boneIndex(a, c.bone)].translation);
}

inline math::Vec3 bonePosition(const SceneActor& a, const std::string& bone)
{
    return worldOf(a)[boneIndex(a, bone)].translation;
}

// Length of one fully stretched arm (upper arm plus forearm), meters.
inline double armLengthM(const SceneActor& a)
{
    return (kUpperArmLengthM + kForearmLengthM) * a.body.scale;
}

// World height of the shoulders.
inline double shoulderHeightM(const SceneActor& a)
{
    return bonePosition(a, "RightUpperArm").z;
}

// Where the actor's pole direction points in the world.
inline math::Vec3 worldPoleOf(const SceneActor& a, const std::string& chain)
{
    for (const markers::IkChain& c : a.body.mapping.ikChains) {
        if (c.name == chain) {
            return math::rotate(a.root.rotation, c.poleDirection);
        }
    }
    return math::Vec3{0.0, 0.0, -1.0};
}

// Poses one arm (side +1 right, -1 left) so its hand is at `handTargetWorld`, as an animation
// clip would. Uses the two-bone IK from A1 and converts its world rotations to local ones.
inline void poseArmToward(SceneActor& a, double side, math::Vec3 handTargetWorld)
{
    const std::string prefix = side > 0.0 ? "Right" : "Left";
    const std::size_t up = boneIndex(a, prefix + "UpperArm");
    const std::size_t fore = boneIndex(a, prefix + "Forearm");
    const std::size_t hand = boneIndex(a, prefix + "Hand");
    const std::vector<math::Transform> w = worldOf(a);
    const ik::TwoBoneChain chain{w[up].translation, w[fore].translation, w[hand].translation};
    const ik::TwoBoneResult r =
        ik::solveTwoBone(chain, handTargetWorld, worldPoleOf(a, prefix + "Arm"),
                         a.body.mapping.ikChains.front().limits);
    const math::Quat parentWorld =
        w[static_cast<std::size_t>(a.body.skeleton.bones[up].parent)].rotation;
    const math::Quat newUpper = math::normalized(r.rootRotation * w[up].rotation);
    const math::Quat newFore = math::normalized(r.midRotation * w[fore].rotation);
    a.pose[up].rotation = math::normalized(math::conjugate(parentWorld) * newUpper);
    a.pose[fore].rotation = math::normalized(math::conjugate(newUpper) * newFore);
}

// A marker whose target point is `worldPoint` for the actors as they stand now: the offset
// is that point expressed in the target region's frame.
inline markers::InteractionMarker makeMarkerAt(const Scene& scene, const std::string& id,
                                               int sourceActor, const std::string& sourceBone,
                                               int targetActor, const std::string& region,
                                               math::Vec3 worldPoint, int priority = 0)
{
    markers::InteractionMarker m;
    m.id = id;
    m.sourceActor = sourceActor;
    m.targetActor = targetActor;
    m.sourceBone = sourceBone;
    m.targetRegion = region;
    const math::Transform frame =
        regionFrame(scene.actors[static_cast<std::size_t>(targetActor)], region);
    m.offsetM = math::transformPoint(math::inverse(frame), worldPoint);
    m.contactDirection = math::Vec3{0.0, 0.0, -1.0};
    m.window = markers::TimeWindow{0.0, 10.0};
    m.priority = priority;
    m.toleranceM = 0.02;
    return m;
}

// Two actors face to face, centres `separationM` apart on the Y axis: actor 0 at the origin
// facing +Y, actor 1 facing -Y.
inline Scene faceToFace(double scale0, double scale1, double separationM)
{
    Scene s;
    s.actors[0] = makeSceneActor(scale0, standingAt(0.0, 0.0, false));
    s.actors[1] = makeSceneActor(scale1, standingAt(0.0, separationM, true));
    return s;
}

// ---- Scenes -----------------------------------------------------------------------------

inline constexpr double kUpperBackRadiusM = 0.075; // at scale 1, see SyntheticHumanoid.hpp

// Point on the back of `target` (the side facing away from `source`), at fraction t up the
// upper-back capsule and swung `swingRad` sideways towards the world direction `lateral`.
inline math::Vec3 backSurfacePoint(const SceneActor& target, double backSignY, double t,
                                   double lateralSignX, double swingRad)
{
    const double r = capsuleRadiusM(target, "UpperBack");
    return capsuleAxisPoint(target, "UpperBack", t) +
           math::Vec3{lateralSignX * r * std::sin(swingRad), backSignY * r * std::cos(swingRad),
                      0.0};
}

// Scene constants. They are scene authoring choices (where on the body the contact is), not
// solver settings.
inline constexpr double kHugGapM = 0.03;            // gap between the two upper backs, scale 1 each
inline constexpr double kHugContactFraction = 0.40; // up the upper-back capsule, from the spine
inline constexpr double kHugSwingRad = 0.314;       // contact swung 18 degrees off the spine line
inline constexpr double kClipHandErrorUpM = 0.10;   // the clip's hand is this far above the contact
inline constexpr double kHandshakeSeparationM = 0.50;
inline constexpr double kHandshakeHeightM = 1.30;
inline constexpr double kShoulderContactFraction = 0.65;
inline constexpr double kShoulderApproachFront = 0.86; // hand comes from the front ...
inline constexpr double kShoulderApproachUp = 0.51;    // ... and from above
inline constexpr double kSceneShiftX = 0.08; // how far actor 1 starts from the reference spot
inline constexpr double kSceneShiftY = 0.15;

// Moves actor 1 away from the reference arrangement, so the solver has a root error to close.
inline void displaceActor1(Scene& scene, double dx, double dy)
{
    scene.actors[1].root.translation = scene.actors[1].root.translation + math::Vec3{dx, dy, 0.0};
}

// (a) Hug: each actor puts its right hand on the other's upper back (the arms pass on opposite
// sides, so they do not cross). The clip's hands are a little above the contacts.
inline Scene makeHugScene(double scale0, double scale1)
{
    const double separation = kUpperBackRadiusM * (scale0 + scale1) + kHugGapM;
    Scene scene = faceToFace(scale0, scale1, separation);
    const Scene reference = scene;
    for (int source = 0; source < 2; ++source) {
        const int target = 1 - source;
        const SceneActor& targetActor = reference.actors[target];
        const double backSignY = target == 1 ? 1.0 : -1.0;  // the back faces away from the source
        const double rightSignX = source == 0 ? 1.0 : -1.0; // the source's right in the world
        const math::Vec3 contact =
            backSurfacePoint(targetActor, backSignY, kHugContactFraction, rightSignX, kHugSwingRad);
        scene.markers.markers.push_back(
            makeMarkerAt(reference, source == 0 ? "actor0RightHandOnBack" : "actor1RightHandOnBack",
                         source, "RightHand", target, "UpperBack", contact));
        poseArmToward(scene.actors[source], 1.0, contact + math::Vec3{0.0, 0.0, kClipHandErrorUpM});
    }
    displaceActor1(scene, kSceneShiftX, kSceneShiftY);
    return scene;
}

// (b) Handshake: right hand to right hand. `source` is the actor whose hand has to move;
// the other actor's hand is the target.
inline Scene makeHandshakeScene(double scale0, double scale1, int source = 0)
{
    Scene scene = faceToFace(scale0, scale1, kHandshakeSeparationM);
    const int target = 1 - source;
    const math::Vec3 meetingPoint{0.0, kHandshakeSeparationM / 2.0, kHandshakeHeightM};
    poseArmToward(scene.actors[target], 1.0, meetingPoint);
    const Scene reference = scene;
    scene.markers.markers.push_back(
        makeMarkerAt(reference, "handshake", source, "RightHand", target, "RightForearm",
                     bonePosition(reference.actors[target], "RightHand")));
    // The source's clip hand is low and short of the meeting point.
    poseArmToward(scene.actors[source], 1.0,
                  meetingPoint + math::Vec3{0.0, source == 0 ? -0.05 : 0.05, -0.12});
    displaceActor1(scene, kSceneShiftX, kSceneShiftY);
    return scene;
}

// (c) A hand on the other actor's shoulder. The taller actor does the touching: a short
// actor cannot reach the top of a much taller one's shoulder without leaning, and the solver
// does not lean actors. The touching hand is the right one, on the target's shoulder on that
// side (the target's left, since the actors face each other).
inline Scene makeShoulderScene(double scale0, double scale1)
{
    const double separation = kUpperBackRadiusM * (scale0 + scale1) + 3.0 * kHugGapM;
    Scene scene = faceToFace(scale0, scale1, separation);
    const Scene reference = scene;
    const int source = scale0 >= scale1 ? 0 : 1;
    const int target = 1 - source;
    const SceneActor& targetActor = reference.actors[target];
    const math::Vec3 contact =
        capsuleAxisPoint(targetActor, "LeftShoulder", kShoulderContactFraction) +
        math::Vec3{0.0, (source == 0 ? -1.0 : 1.0) * kShoulderApproachFront, kShoulderApproachUp} *
            capsuleRadiusM(targetActor, "LeftShoulder");
    scene.markers.markers.push_back(makeMarkerAt(reference, "handOnShoulder", source, "RightHand",
                                                 target, "LeftShoulder", contact));
    poseArmToward(scene.actors[source], 1.0, contact + math::Vec3{0.0, 0.0, kClipHandErrorUpM});
    displaceActor1(scene, kSceneShiftX, kSceneShiftY);
    return scene;
}

inline constexpr double kPenetrationSeparationM = 0.30;

// Penetration avoidance: actor 0's right hand goes to actor 1's far (right) shoulder, across
// its chest, and actor 0's right arm has its pole pointing forward (+Y, towards the partner).
// With only that pole, the elbow and forearm run through the partner's body; trying other
// pole directions around the shoulder-to-target axis finds a way around.
inline Scene makePenetrationScene()
{
    Scene scene = faceToFace(1.0, 1.0, kPenetrationSeparationM);
    const Scene reference = scene;
    const SceneActor& target = reference.actors[1];
    const math::Vec3 contact =
        capsuleAxisPoint(target, "RightShoulder", 0.65) +
        math::normalized(math::Vec3{0.0, -0.6, 0.8}) * capsuleRadiusM(target, "RightShoulder");
    scene.markers.markers.push_back(makeMarkerAt(reference, "handAcrossToShoulder", 0, "RightHand",
                                                 1, "RightShoulder", contact));
    // The clip was made with the ordinary relaxed pole; only afterwards is the pole turned
    // towards the partner.
    poseArmToward(scene.actors[0], 1.0, contact + math::Vec3{0.0, 0.0, kClipHandErrorUpM});
    scene.actors[0].body.mapping.ikChains[0].poleDirection = math::Vec3{0.0, 1.0, 0.0};
    displaceActor1(scene, kSceneShiftX, kSceneShiftY);
    return scene;
}

} // namespace opas::test
