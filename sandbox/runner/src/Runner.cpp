#include "opas/sandbox/Runner.hpp"

#include "opas/physics/jolt/JoltWorld.hpp"
#include "opas/rig/ProfileFile.hpp"
#include "opas/rig/RigFile.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

namespace opas::sandbox {
namespace {

// Worlds are sized to the scenario; one spare body and joint keep the
// capacity above zero when a scenario has no statics or no joints.
constexpr std::uint32_t kSpareCapacity = 1;

// M3's arm wave (docs/tasks/0013): 0.5 Hz, shoulder +-0.6 rad, elbow
// 0.75 +- 0.45 rad, both about +X.
constexpr double kWaveHz = 0.5;
constexpr double kShoulderAmplitudeRad = 0.6;
constexpr double kElbowMeanRad = 0.75;
constexpr double kElbowAmplitudeRad = 0.45;

std::vector<math::Transform> restPose(const rig::Rig& rig)
{
    std::vector<math::Transform> pose(rig.skeleton.bones.size());
    for (std::size_t i = 0; i < pose.size(); ++i) {
        pose[i] = rig.skeleton.bones[i].localRest;
    }
    return pose;
}

format::LoadError errorAt(const RunOptions& options, std::size_t line, std::string field,
                          std::string message)
{
    return format::LoadError{options.scenarioFile, line == 0 ? 1 : line, std::move(field),
                             std::move(message)};
}

physics::Vec3 toPhysics(math::Vec3 v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

physics::Quat toPhysics(math::Quat q)
{
    const math::Quat n = math::normalized(q);
    return {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z),
            static_cast<float>(n.w)};
}

BodySample sampleOf(const physics::BodyState& s)
{
    return {s.positionM.x,
            s.positionM.y,
            s.positionM.z,
            s.rotation.x,
            s.rotation.y,
            s.rotation.z,
            s.rotation.w,
            s.linearVelocityMps.x,
            s.linearVelocityMps.y,
            s.linearVelocityMps.z,
            s.angularVelocityRadps.x,
            s.angularVelocityRadps.y,
            s.angularVelocityRadps.z};
}

// A ragdoll in the run, with what it needs to report errors and impulses.
struct RunRagdoll {
    std::string name;
    rig::Rig rig;
    motor::Ragdoll ragdoll;
    std::unique_ptr<motor::RagdollDriver> driver;
};

} // namespace

bool isPoseSourceName(std::string_view name)
{
    return name == "rest" || name == "armWave";
}

std::optional<motor::PoseSource> makePoseSource(std::string_view name, const rig::Rig& rig,
                                                std::string* error)
{
    const std::vector<math::Transform> rest = restPose(rig);
    if (name == "rest") {
        // Copying into a span the driver sized to the skeleton cannot throw.
        // NOLINTNEXTLINE(bugprone-exception-escape)
        return motor::PoseSource([rest](double /*timeSeconds*/, std::span<math::Transform> pose) {
            std::copy(rest.begin(), rest.end(), pose.begin());
        });
    }
    if (name == "armWave") {
        const auto shoulder = rig::findBone(rig.skeleton, "RightUpperArm");
        const auto elbow = rig::findBone(rig.skeleton, "RightForearm");
        if (!shoulder || !elbow) {
            if (error != nullptr) {
                *error = "pose source \"armWave\" needs bones RightUpperArm and RightForearm";
            }
            return std::nullopt;
        }
        // As above: no allocation, nothing that throws.
        return motor::PoseSource(
            // NOLINTNEXTLINE(bugprone-exception-escape)
            [rest, s = *shoulder, e = *elbow](double timeSeconds, std::span<math::Transform> pose) {
                std::copy(rest.begin(), rest.end(), pose.begin());
                const double phase = 2.0 * std::numbers::pi * kWaveHz * timeSeconds;
                const math::Vec3 x{1.0, 0.0, 0.0};
                pose[s].rotation = math::fromAxisAngle(x, kShoulderAmplitudeRad * std::sin(phase));
                pose[e].rotation =
                    math::fromAxisAngle(x, kElbowMeanRad + kElbowAmplitudeRad * std::sin(phase));
            });
    }
    if (error != nullptr) {
        *error = "unknown pose source \"" + std::string(name) + "\"";
    }
    return std::nullopt;
}

RunResult run(const Scenario& scenario, const RunOptions& options)
{
    RunResult result;

    // --- Load every ragdoll's rig and profile first, so errors come before
    // anything is built.
    std::vector<RunRagdoll> ragdolls(scenario.ragdolls.size());
    std::vector<rig::Profile> profiles(scenario.ragdolls.size());
    std::uint32_t bodyCount = static_cast<std::uint32_t>(scenario.statics.size()) + kSpareCapacity;
    std::uint32_t jointCount = kSpareCapacity;
    for (std::size_t r = 0; r < scenario.ragdolls.size(); ++r) {
        const ScenarioRagdoll& entry = scenario.ragdolls[r];
        const std::string at = "/ragdolls/" + std::to_string(r);
        const auto rig = rig::loadRigFile(options.baseDir / entry.rigPath);
        if (!rig.value) {
            result.error = errorAt(options, entry.line, at + "/rig", rig.error.toString());
            return result;
        }
        const auto profile = rig::loadProfileFile(options.baseDir / entry.profilePath);
        if (!profile.value) {
            result.error = errorAt(options, entry.line, at + "/profile", profile.error.toString());
            return result;
        }
        ragdolls[r].name = entry.name;
        ragdolls[r].rig = *rig.value;
        profiles[r] = *profile.value;
        if (entry.motors == MotorMode::Off) {
            // Limp: no pull towards the pose, damping kept.
            profiles[r].defaultMotor.stiffnessNmPerRad = 0.0;
            for (rig::JointOverride& joint : profiles[r].joints) {
                joint.motor.stiffnessNmPerRad = 0.0;
            }
        }
        bodyCount += static_cast<std::uint32_t>(ragdolls[r].rig.bodies.size());
        jointCount += static_cast<std::uint32_t>(ragdolls[r].rig.joints.size());
    }

    // Impulses name a bone with a body.
    for (std::size_t i = 0; i < scenario.impulses.size(); ++i) {
        const Impulse& impulse = scenario.impulses[i];
        const auto owner = std::find_if(ragdolls.begin(), ragdolls.end(), [&](const RunRagdoll& r) {
            return r.name == impulse.ragdoll;
        });
        if (owner == ragdolls.end() || !rig::findBody(owner->rig, impulse.bone)) {
            result.error = errorAt(
                options, impulse.line, "/impulses/" + std::to_string(i) + "/bone",
                "ragdoll \"" + impulse.ragdoll + "\" has no body on bone \"" + impulse.bone + "\"");
            return result;
        }
    }

    // --- Build the world.
    physics::WorldConfig config;
    config.fixedStepSeconds = 1.0f / static_cast<float>(scenario.stepsPerSecond);
    config.gravityMps2 = toPhysics(scenario.gravityMps2);
    // Collision groups have their own table, capped at maxBodies (one per
    // ragdoll here), so the body count alone is enough.
    config.maxBodies = bodyCount;
    config.maxJoints = jointCount;
    std::string worldError;
    const std::unique_ptr<physics::PhysicsWorld> world =
        physics::jolt::createJoltWorld(config, &worldError);
    if (world == nullptr) {
        result.error = errorAt(options, 1, "", "cannot create the physics world: " + worldError);
        return result;
    }

    for (std::size_t s = 0; s < scenario.statics.size(); ++s) {
        const StaticBox& box = scenario.statics[s];
        physics::BodyDesc desc;
        desc.shape = physics::BoxShape{toPhysics(box.halfExtentsM)};
        desc.motion = physics::MotionType::Static;
        desc.positionM = toPhysics(box.pose.translation);
        desc.rotation = toPhysics(box.pose.rotation);
        desc.friction = static_cast<float>(box.friction);
        const auto created = world->createBody(desc);
        if (!created.ok()) {
            result.error =
                errorAt(options, box.line, "/statics/" + std::to_string(s), created.message);
            return result;
        }
    }

    Recording& recording = result.recording.emplace();
    recording.scenario = scenario.name;
    recording.stepsPerSecond = scenario.stepsPerSecond;
    std::vector<physics::BodyId> recorded;
    for (std::size_t r = 0; r < ragdolls.size(); ++r) {
        const ScenarioRagdoll& entry = scenario.ragdolls[r];
        const std::string at = "/ragdolls/" + std::to_string(r);
        motor::BuildResult built =
            motor::buildRagdoll(*world, ragdolls[r].rig, profiles[r], entry.root,
                                motor::RagdollOptions{entry.rootMotion});
        if (!built.ragdoll) {
            result.recording.reset();
            result.error = errorAt(options, entry.line, at, built.message);
            return result;
        }
        ragdolls[r].ragdoll = std::move(*built.ragdoll);
        std::string poseError;
        std::optional<motor::PoseSource> source =
            makePoseSource(entry.pose, ragdolls[r].rig, &poseError);
        if (!source) {
            result.recording.reset();
            result.error = errorAt(options, entry.line, at + "/pose", poseError);
            return result;
        }
        // The driver points at ragdolls[r].ragdoll; the vector is not resized
        // again, so the pointer stays valid for the run.
        ragdolls[r].driver =
            std::make_unique<motor::RagdollDriver>(ragdolls[r].ragdoll, std::move(*source));
        world->addStepListener(*ragdolls[r].driver);
        for (std::size_t b = 0; b < ragdolls[r].rig.bodies.size(); ++b) {
            recorded.push_back(ragdolls[r].ragdoll.bodies[b].id);
            recording.bodies.push_back(entry.name + "/" + ragdolls[r].rig.bodies[b].bone);
        }
    }

    // --- Step and record.
    const int steps = stepCount(scenario);
    std::vector<physics::BodyState> states(recorded.size());
    const auto record = [&] {
        world->readBodies(recorded, states);
        std::vector<BodySample>& samples = recording.steps.emplace_back();
        samples.reserve(states.size());
        for (const physics::BodyState& state : states) {
            samples.push_back(sampleOf(state));
        }
    };
    recording.steps.reserve(static_cast<std::size_t>(steps) + 1);
    record();
    for (int step = 0; step < steps; ++step) {
        for (const Impulse& impulse : scenario.impulses) {
            if (impulseStep(scenario, impulse.atS) != step) {
                continue;
            }
            const auto owner =
                std::find_if(ragdolls.begin(), ragdolls.end(),
                             [&](const RunRagdoll& r) { return r.name == impulse.ragdoll; });
            const std::size_t body = rig::findBody(owner->rig, impulse.bone).value_or(0);
            const physics::BodyId id = owner->ragdoll.bodies[body].id;
            physics::BodyState state;
            world->readBody(id, state);
            const physics::Vec3 point{state.positionM.x + static_cast<float>(impulse.offsetM.x),
                                      state.positionM.y + static_cast<float>(impulse.offsetM.y),
                                      state.positionM.z + static_cast<float>(impulse.offsetM.z)};
            world->applyImpulse(id, toPhysics(impulse.impulseNs), point);
        }
        world->step();
        record();
    }
    return result;
}

RunResult runFile(const std::filesystem::path& scenarioPath)
{
    RunResult result;
    const ScenarioLoadResult scenario = loadScenarioFile(scenarioPath);
    if (!scenario.value) {
        result.error = scenario.error;
        return result;
    }
    const std::u8string file = scenarioPath.filename().u8string();
    return run(*scenario.value,
               RunOptions{scenarioPath.parent_path(), std::string(file.begin(), file.end())});
}

} // namespace opas::sandbox
