#include "CountingNew.hpp"

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

// Replacing the global allocation functions is allowed by the standard
// ([replacement.functions]); every form is replaced so none slips past the
// counter. Aligned forms need the matching aligned free on MSVC. This file
// is raw memory management by nature, so those checks are off for all of it.
// NOLINTBEGIN(cppcoreguidelines-no-malloc, cppcoreguidelines-owning-memory)

namespace {

std::atomic<bool> counting{false};
std::atomic<std::uint64_t> allocations{0};

void* allocate(std::size_t size)
{
    if (counting.load(std::memory_order_relaxed)) {
        allocations.fetch_add(1, std::memory_order_relaxed);
    }
    return std::malloc(size == 0 ? 1 : size);
}

void* allocateAligned(std::size_t size, std::align_val_t alignment)
{
    if (counting.load(std::memory_order_relaxed)) {
        allocations.fetch_add(1, std::memory_order_relaxed);
    }
    const auto align = static_cast<std::size_t>(alignment);
#if defined(_MSC_VER)
    return _aligned_malloc(size == 0 ? 1 : size, align);
#else
    // aligned_alloc needs a size that is a multiple of the alignment.
    const std::size_t rounded = ((size == 0 ? 1 : size) + align - 1) / align * align;
    return std::aligned_alloc(align, rounded);
#endif
}

void freeAligned(void* p)
{
#if defined(_MSC_VER)
    _aligned_free(p);
#else
    std::free(p);
#endif
}

void* orThrow(void* p)
{
    if (p == nullptr) {
        throw std::bad_alloc();
    }
    return p;
}

} // namespace

namespace opas::test {

void startCountingAllocations()
{
    allocations.store(0);
    counting.store(true);
}

std::uint64_t stopCountingAllocations()
{
    counting.store(false);
    return allocations.load();
}

} // namespace opas::test

void* operator new(std::size_t size)
{
    return orThrow(allocate(size));
}
void* operator new[](std::size_t size)
{
    return orThrow(allocate(size));
}
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept
{
    return allocate(size);
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept
{
    return allocate(size);
}
void* operator new(std::size_t size, std::align_val_t alignment)
{
    return orThrow(allocateAligned(size, alignment));
}
void* operator new[](std::size_t size, std::align_val_t alignment)
{
    return orThrow(allocateAligned(size, alignment));
}
void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t& /*tag*/) noexcept
{
    return allocateAligned(size, alignment);
}
void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t& /*tag*/) noexcept
{
    return allocateAligned(size, alignment);
}

void operator delete(void* p) noexcept
{
    std::free(p);
}
void operator delete[](void* p) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::size_t /*size*/) noexcept
{
    std::free(p);
}
void operator delete[](void* p, std::size_t /*size*/) noexcept
{
    std::free(p);
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept
{
    std::free(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::align_val_t /*alignment*/) noexcept
{
    freeAligned(p);
}
void operator delete[](void* p, std::align_val_t /*alignment*/) noexcept
{
    freeAligned(p);
}
void operator delete(void* p, std::size_t /*size*/, std::align_val_t /*alignment*/) noexcept
{
    freeAligned(p);
}
void operator delete[](void* p, std::size_t /*size*/, std::align_val_t /*alignment*/) noexcept
{
    freeAligned(p);
}
void operator delete(void* p, std::align_val_t /*alignment*/,
                     const std::nothrow_t& /*tag*/) noexcept
{
    freeAligned(p);
}
void operator delete[](void* p, std::align_val_t /*alignment*/,
                       const std::nothrow_t& /*tag*/) noexcept
{
    freeAligned(p);
}
// NOLINTEND(cppcoreguidelines-no-malloc, cppcoreguidelines-owning-memory)
