#include "opas/physics/jolt/BackendInfo.hpp"

// Jolt requires Jolt.h before any other Jolt header (see HelloWorld.cpp in the
// Jolt repository).
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

#include <memory>

namespace opas::physics::jolt {

JoltVersion linkedJoltVersion()
{
    return JoltVersion{JPH_VERSION_MAJOR, JPH_VERSION_MINOR, JPH_VERSION_PATCH};
}

std::size_t countRegisteredJoltTypes()
{
    JPH::RegisterDefaultAllocator();

    // Jolt keeps the factory in a global raw pointer; we own it here and clear
    // the global before the factory is destroyed.
    const auto factory = std::make_unique<JPH::Factory>();
    JPH::Factory::sInstance = factory.get();
    JPH::RegisterTypes();

    const std::size_t count = factory->GetAllClasses().size();

    JPH::UnregisterTypes();
    JPH::Factory::sInstance = nullptr;
    return count;
}

} // namespace opas::physics::jolt
