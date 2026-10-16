#pragma once

// Small helpers shared by the Jolt backend tests. Standard library only, so
// the tests depend on nothing but the physics interface and the factory.
//
// Quaternion math is done in double so that the helper's own rounding never
// eats into the tolerances being tested.

#include "opas/physics/PhysicsWorld.hpp"
#include "opas/physics/jolt/JoltWorld.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>

namespace opas::physics::test {

constexpr float kPi = std::numbers::pi_v<float>;

constexpr float degToRad(float degrees)
{
    return degrees * kPi / 180.0f;
}

constexpr double radToDeg(double radians)
{
    return radians * 180.0 / std::numbers::pi;
}

// Fixed step count for a duration at the default 60 Hz, e.g. 3 s -> 180.
constexpr int kStepsPerSecond = 60;

inline std::unique_ptr<PhysicsWorld> makeWorld(const WorldConfig& config = {})
{
    return jolt::createJoltWorld(config);
}

inline WorldConfig zeroGravityConfig()
{
    WorldConfig config;
    config.gravityMps2 = Vec3{0.0f, 0.0f, 0.0f};
    return config;
}

// Test choice: the floor is a 100 m x 100 m static box, 1 m thick, whose top
// face is exactly z = 0 (centre at z = -0.5).
inline BodyId createFloor(PhysicsWorld& world)
{
    BodyDesc desc;
    desc.shape = BoxShape{Vec3{50.0f, 50.0f, 0.5f}};
    desc.motion = MotionType::Static;
    desc.positionM = Vec3{0.0f, 0.0f, -0.5f};
    return world.createBody(desc).id;
}

inline BodyDesc dynamicBox(Vec3 halfExtentsM, float massKg, Vec3 positionM, Quat rotation = {})
{
    BodyDesc desc;
    desc.shape = BoxShape{halfExtentsM};
    desc.motion = MotionType::Dynamic;
    desc.massKg = massKg;
    desc.positionM = positionM;
    desc.rotation = rotation;
    return desc;
}

// Two bodies ready to be jointed, used by the joint, handle and listener tests.
struct JointedPair {
    BodyId anchor;       // static
    BodyId box;          // dynamic, 1 kg, half extents 0.1 m
    JointDesc jointDesc; // hinge-like: zero cones, twist limits as requested
};

// Test choice: the static anchor is a 0.2 m cube at the origin and the 1 kg
// box (half extents 0.1 m) sits 0.3 m along +X, with the joint anchor halfway
// between. Twist axis +X runs through both centres, so twisting does not move
// the box's centre, and with plane axis +Y the joint frame equals the world
// frame at creation (both bodies start unrotated).
inline JointedPair createJointedPair(PhysicsWorld& world, float twistMinRad, float twistMaxRad)
{
    JointedPair pair;

    BodyDesc anchorDesc;
    anchorDesc.shape = BoxShape{Vec3{0.1f, 0.1f, 0.1f}};
    anchorDesc.motion = MotionType::Static;
    pair.anchor = world.createBody(anchorDesc).id;

    pair.box =
        world.createBody(dynamicBox(Vec3{0.1f, 0.1f, 0.1f}, 1.0f, Vec3{0.3f, 0.0f, 0.0f})).id;

    pair.jointDesc.bodyA = pair.anchor;
    pair.jointDesc.bodyB = pair.box;
    pair.jointDesc.anchorM = Vec3{0.15f, 0.0f, 0.0f};
    pair.jointDesc.twistAxis = Vec3{1.0f, 0.0f, 0.0f};
    pair.jointDesc.planeAxis = Vec3{0.0f, 1.0f, 0.0f};
    pair.jointDesc.normalHalfConeRad = 0.0f;
    pair.jointDesc.planeHalfConeRad = 0.0f;
    pair.jointDesc.twistMinRad = twistMinRad;
    pair.jointDesc.twistMaxRad = twistMaxRad;
    return pair;
}

// --- Vector and quaternion helpers -------------------------------------------
// A unit quaternion q = (v, w) with v = axis * sin(angle / 2) and
// w = cos(angle / 2) rotates by `angle` about `axis`.

struct DQuat {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct DVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline DQuat toDouble(Quat q)
{
    return DQuat{q.x, q.y, q.z, q.w};
}

inline DVec3 toDouble(Vec3 v)
{
    return DVec3{v.x, v.y, v.z};
}

inline double length(Vec3 v)
{
    const DVec3 d = toDouble(v);
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

inline double distance(Vec3 a, Vec3 b)
{
    const DVec3 da = toDouble(a);
    const DVec3 db = toDouble(b);
    const double dx = da.x - db.x;
    const double dy = da.y - db.y;
    const double dz = da.z - db.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Rotation of `angleRad` about the unit vector `axis`.
inline Quat axisAngle(Vec3 axis, float angleRad)
{
    const double half = static_cast<double>(angleRad) * 0.5;
    const double s = std::sin(half);
    return Quat{static_cast<float>(axis.x * s), static_cast<float>(axis.y * s),
                static_cast<float>(axis.z * s), static_cast<float>(std::cos(half))};
}

// The conjugate (-v, w) of a unit quaternion is its inverse rotation.
inline DQuat conjugate(DQuat q)
{
    return DQuat{-q.x, -q.y, -q.z, q.w};
}

// Hamilton product: (a * b) applies b first, then a.
inline DQuat multiply(DQuat a, DQuat b)
{
    return DQuat{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                 a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                 a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                 a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// Rotates v by q: v' = q * (v, 0) * conj(q).
inline DVec3 rotate(DQuat q, DVec3 v)
{
    const DQuat p = multiply(multiply(q, DQuat{v.x, v.y, v.z, 0.0}), conjugate(q));
    return DVec3{p.x, p.y, p.z};
}

// Angle of the rotation that takes a to b, in [0, pi].
// The relative rotation is r = conj(a) * b = (v, w), and its angle is
// 2 * atan2(|v|, |w|). Using |w| picks the shorter path (q and -q are the same
// rotation). atan2 stays accurate for tiny angles, where 2 * acos(w) would
// lose almost all precision because acos is very steep near 1.
inline double angleBetween(Quat a, Quat b)
{
    const DQuat r = multiply(conjugate(toDouble(a)), toDouble(b));
    const double vecLength = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    return 2.0 * std::atan2(vecLength, std::abs(r.w));
}

// Angle between the body's local +Z axis (rotated into the world) and world +Z.
inline double tiltRad(Quat rotation)
{
    const DVec3 up = rotate(toDouble(rotation), DVec3{0.0, 0.0, 1.0});
    const double horizontal = std::sqrt(up.x * up.x + up.y * up.y);
    return std::atan2(horizontal, up.z);
}

// Twist angle of body B relative to body A about `twistAxisInA`, in (-pi, pi].
//
// qRel = conj(qA) * qB is B's rotation expressed in A's frame. Swing-twist
// decomposition splits qRel = swing * twist, where twist rotates about the
// twist axis and swing about an axis perpendicular to it. The twist part is
// found by projecting qRel's vector part onto the axis: twist = (axis *
// dot(v, axis), w), normalized. Its angle is therefore
// 2 * atan2(dot(v, axis), w). Flipping qRel to w >= 0 first makes the result
// the same for q and -q.
inline double twistAngleRad(Quat qA, Quat qB, Vec3 twistAxisInA)
{
    DQuat r = multiply(conjugate(toDouble(qA)), toDouble(qB));
    if (r.w < 0.0) {
        r = DQuat{-r.x, -r.y, -r.z, -r.w};
    }
    const DVec3 axis = toDouble(twistAxisInA);
    const double projection = r.x * axis.x + r.y * axis.y + r.z * axis.z;
    return 2.0 * std::atan2(projection, r.w);
}

// --- Bit-exact comparison ------------------------------------------------------
// Determinism means the exact same bits, not "close enough". Comparing the bit
// patterns also treats -0.0f and 0.0f as different, which == would not.

inline bool bitEqual(float a, float b)
{
    return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
}

inline bool bitEqual(Vec3 a, Vec3 b)
{
    return bitEqual(a.x, b.x) && bitEqual(a.y, b.y) && bitEqual(a.z, b.z);
}

inline bool bitEqual(Quat a, Quat b)
{
    return bitEqual(a.x, b.x) && bitEqual(a.y, b.y) && bitEqual(a.z, b.z) && bitEqual(a.w, b.w);
}

// Field by field rather than memcmp of the whole struct: BodyState has padding
// after `awake`, and padding bytes are not guaranteed to match.
inline bool bitEqual(const BodyState& a, const BodyState& b)
{
    return bitEqual(a.positionM, b.positionM) && bitEqual(a.rotation, b.rotation) &&
           bitEqual(a.linearVelocityMps, b.linearVelocityMps) &&
           bitEqual(a.angularVelocityRadps, b.angularVelocityRadps) && a.awake == b.awake;
}

} // namespace opas::physics::test
