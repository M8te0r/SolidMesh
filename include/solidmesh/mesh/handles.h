#pragma once
#include <cstdint>
#include <limits>

namespace SolidMesh {

constexpr uint32_t INVALID_SLOT = std::numeric_limits<uint32_t>::max();

// Generation-based stable handles. slot+generation together uniquely identify
// a live entity; a stale handle (pointing to a recycled slot) is detectable
// because the generation won't match.

#define SM_DEFINE_ID(Name)                                                                      \
    struct Name {                                                                               \
        uint32_t slot       = INVALID_SLOT;                                                     \
        uint32_t generation = 0;                                                                \
        bool is_valid() const noexcept { return slot != INVALID_SLOT; }                         \
        uint64_t value() const noexcept{return (static_cast<uint64_t>(generation)<<32)|slot;}   \
        static Name from_value(uint64_t v) noexcept {                                           \
            return Name{                                                                        \
                static_cast<uint32_t>(v & 0xffffffffull),                                       \
                static_cast<uint32_t>(v >> 32)                                                  \
            };                                                                                  \
        }                                                                                       \
        bool operator==(const Name& o) const noexcept {                                         \
            return slot == o.slot && generation == o.generation;                                \
        }                                                                                       \
        bool operator!=(const Name& o) const noexcept {                                         \
            return !(*this == o);                                                               \
        }                                                                                       \
    };

SM_DEFINE_ID(VertexID)
SM_DEFINE_ID(FaceID)
SM_DEFINE_ID(HalfFaceID)
SM_DEFINE_ID(CellID)

#undef SM_DEFINE_ID

} // namespace SolidMesh
