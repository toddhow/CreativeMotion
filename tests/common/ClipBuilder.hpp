#pragma once

// Builders for pose clips used by the contact detection tests (A4a): a tiny two-actor rig for
// hand-computed rule cases, and a converter from the synthetic humanoid scenes (clips of
// SceneActor frames). No game data.

#include "AlignmentScenes.hpp"
#include "SyntheticHumanoid.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace opas::test {

// ---- Mini rig for hand-computed rules -----------------------------------------------------------
//
// Both actors have the same six bones and body map:
//   limb "Arm":  Base, Elbow, <hand>      limb "Arm2": Base, Hand2
//   capsule <region> ("Post"): Base -> Tip, radius = ratio * length
//   capsule "Rod":              Base -> Side
// Source bones are therefore <hand> (default "Hand") and "Hand2". Unless a test moves them,
// every bone sits far from every other actor's capsules (tens of meters), so a test only has to
// place the bones it is about.
//
// Actor 1's capsules (the target of actor 0's hand): Base at the origin, Tip at (0,0,1), Side at
// (1,0,0), so with ratio 0.1 the post is the segment (0,0,0)-(0,0,1) with radius 0.1 m.
// Actor 0's own capsules sit 100 m away in X (Base (100,0,0), Tip (100,0,1), Side (101,0,0)),
// so actor 1's hand can touch them by being placed at x = 100 + something.

struct MiniNames {
    std::string hand = "Hand";
    std::string region = "Post"; // the capsule Base -> Tip
};

inline markers::BodyMapping miniBodyMap(double radiusRatio = 0.1, const MiniNames& names = {})
{
    markers::BodyMapping m;
    m.limbs = {{"Arm", {"Base", "Elbow", names.hand}}, {"Arm2", {"Base", "Hand2"}}};
    m.capsules = {{names.region, "Base", "Tip", radiusRatio}, {"Rod", "Base", "Side", radiusRatio}};
    return m;
}

inline std::array<markers::BodyMapping, 2> miniBodyMaps(double radiusRatio = 0.1,
                                                        const MiniNames& names = {})
{
    return {miniBodyMap(radiusRatio, names), miniBodyMap(radiusRatio, names)};
}

class MiniClip {
public:
    MiniClip(double sampleRateHz, std::size_t frameCount, const MiniNames& names = {})
    {
        clip_.sampleRateHz = sampleRateHz;
        const std::vector<std::pair<std::string, math::Vec3>> defaults0 = {
            {"Base", {100, 0, 0}},      {"Tip", {100, 0, 1}},   {"Side", {101, 0, 0}},
            {names.hand, {50, -50, 0}}, {"Elbow", {50, 50, 0}}, {"Hand2", {50, -60, 0}}};
        const std::vector<std::pair<std::string, math::Vec3>> defaults1 = {
            {"Base", {0, 0, 0}},         {"Tip", {0, 0, 1}},      {"Side", {1, 0, 0}},
            {names.hand, {-50, -50, 0}}, {"Elbow", {-50, 50, 0}}, {"Hand2", {-50, -60, 0}}};
        for (std::size_t a = 0; a < 2; ++a) {
            for (const auto& [name, position] : a == 0 ? defaults0 : defaults1) {
                clip_.actors[a].bones.push_back(name);
            }
            clip_.actors[a].bodyMap = "mini";
            for (std::size_t f = 0; f < frameCount; ++f) {
                std::vector<math::Transform> frame;
                for (const auto& [name, position] : a == 0 ? defaults0 : defaults1) {
                    frame.push_back(math::Transform{position, math::Quat{}});
                }
                clip_.actors[a].frames.push_back(frame);
            }
        }
    }

    // Sets one bone's world transform in one frame.
    MiniClip& set(std::size_t actor, const std::string& bone, std::size_t frame, math::Transform t)
    {
        clip_.actors[actor].frames[frame][indexOf(actor, bone)] = t;
        return *this;
    }
    MiniClip& set(std::size_t actor, const std::string& bone, std::size_t frame,
                  math::Vec3 position)
    {
        return set(actor, bone, frame, math::Transform{position, math::Quat{}});
    }
    // Sets one bone's transform in every frame.
    MiniClip& setAll(std::size_t actor, const std::string& bone, math::Transform t)
    {
        for (std::size_t f = 0; f < clip_.actors[actor].frames.size(); ++f) {
            set(actor, bone, f, t);
        }
        return *this;
    }
    // Places actor 0's hand frame by frame (the usual way to script a contact).
    MiniClip& hand0(const std::vector<math::Vec3>& positions, const std::string& bone = "Hand")
    {
        for (std::size_t f = 0; f < positions.size(); ++f) {
            set(0, bone, f, positions[f]);
        }
        return *this;
    }
    // Removes a bone (and its column of every frame) from an actor.
    MiniClip& removeBone(std::size_t actor, const std::string& bone)
    {
        const std::size_t i = indexOf(actor, bone);
        clip_.actors[actor].bones.erase(clip_.actors[actor].bones.begin() +
                                        static_cast<std::ptrdiff_t>(i));
        for (auto& frame : clip_.actors[actor].frames) {
            frame.erase(frame.begin() + static_cast<std::ptrdiff_t>(i));
        }
        return *this;
    }

    [[nodiscard]] const markers::PoseClip& clip() const { return clip_; }
    markers::PoseClip& clip() { return clip_; }

private:
    [[nodiscard]] std::size_t indexOf(std::size_t actor, const std::string& bone) const
    {
        const auto& bones = clip_.actors[actor].bones;
        for (std::size_t i = 0; i < bones.size(); ++i) {
            if (bones[i] == bone) {
                return i;
            }
        }
        return 0; // the tests only name bones that exist
    }

    markers::PoseClip clip_;
};

// ---- Clips of the synthetic humanoid scenes ----------------------------------------------------

// One moment of a two-actor scene as a clip frame: every bone's world transform.
inline void appendFrame(markers::PoseClip& clip, const SceneActor& a0, const SceneActor& a1)
{
    const SceneActor* actors[2] = {&a0, &a1};
    for (std::size_t a = 0; a < 2; ++a) {
        markers::ClipActor& target = clip.actors[a];
        if (target.bones.empty()) {
            target.bodyMap = "humanoid";
            for (const rig::Bone& b : actors[a]->body.skeleton.bones) {
                target.bones.push_back(b.name);
            }
        }
        target.frames.push_back(worldOf(*actors[a]));
    }
}

// The local pose that reproduces given world transforms with an identity root: the way the
// alignment solver is fed from a clip frame (the plugin's recordings hold world transforms).
inline std::vector<math::Transform> localPoseFromWorld(const rig::Skeleton& skeleton,
                                                       const std::vector<math::Transform>& world)
{
    std::vector<math::Transform> local(world.size());
    for (std::size_t i = 0; i < world.size(); ++i) {
        const int parent = skeleton.bones[i].parent;
        local[i] = parent < 0 ? world[i]
                              : math::inverse(world[static_cast<std::size_t>(parent)]) * world[i];
    }
    return local;
}

} // namespace opas::test
