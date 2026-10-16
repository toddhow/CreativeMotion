// Includes every physics/interface header in a target that links only
// opas::physics_interface. If any header pulled in Jolt (or anything outside
// the standard library), this file would fail to compile or link.

#include "opas/physics/Descriptions.hpp"
#include "opas/physics/PhysicsWorld.hpp"
#include "opas/physics/Types.hpp"

#include <gtest/gtest.h>

#include <type_traits>

namespace {

using opas::physics::PhysicsWorld;
using opas::physics::StepListener;

// The world is owned through std::unique_ptr and handed around by reference;
// copying or moving one would duplicate or orphan engine state.
static_assert(std::is_abstract_v<PhysicsWorld>);
static_assert(!std::is_copy_constructible_v<PhysicsWorld>);
static_assert(!std::is_copy_assignable_v<PhysicsWorld>);
static_assert(!std::is_move_constructible_v<PhysicsWorld>);
static_assert(!std::is_move_assignable_v<PhysicsWorld>);
static_assert(std::has_virtual_destructor_v<PhysicsWorld>);

static_assert(std::is_abstract_v<StepListener>);
static_assert(std::has_virtual_destructor_v<StepListener>);

TEST(PhysicsInterfaceHeaders, AllHeadersCompileWithoutJolt)
{
    // The real check is that this translation unit compiled and linked.
    SUCCEED();
}

} // namespace
