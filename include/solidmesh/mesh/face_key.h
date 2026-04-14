#pragma once
#include <vector>
#include <functional>
#include <algorithm>
#include "solidmesh/mesh/handles.h"

namespace SolidMesh {

// Canonical, unoriented key for a polygonal face.
// Two halffaces sharing the same geometric face produce the same FaceKey
// regardless of orientation or starting vertex.
//
// Algorithm: among all rotations of the forward ring and all rotations of the
// reversed ring, pick the lexicographically smallest sequence of vertex slots.
struct FaceKey {
    std::vector<uint32_t> slots;  // vertex slots in canonical order

    bool operator==(const FaceKey& o) const noexcept { return slots == o.slots; }
    bool operator!=(const FaceKey& o) const noexcept { return !(*this == o); }
};

inline FaceKey make_face_key(const std::vector<VertexID>& verts) {
    const int n = static_cast<int>(verts.size());
    std::vector<uint32_t> best;
    best.reserve(n);

    auto try_rotation = [&](const std::vector<uint32_t>& ring, int start) {
        std::vector<uint32_t> candidate;
        candidate.reserve(n);
        for (int i = 0; i < n; ++i)
            candidate.push_back(ring[(start + i) % n]);
        if (best.empty() || candidate < best)
            best = std::move(candidate);
    };

    // Forward ring
    std::vector<uint32_t> fwd(n);
    for (int i = 0; i < n; ++i) fwd[i] = verts[i].slot;

    // Reverse ring
    std::vector<uint32_t> rev(n);
    for (int i = 0; i < n; ++i) rev[i] = verts[n - 1 - i].slot;

    for (int i = 0; i < n; ++i) {
        try_rotation(fwd, i);
        try_rotation(rev, i);
    }

    return FaceKey{std::move(best)};
}

struct FaceKeyHash {
    size_t operator()(const FaceKey& k) const noexcept {
        // FNV-1a over the slot array
        size_t h = 14695981039346656037ULL;
        for (uint32_t s : k.slots) {
            h ^= static_cast<size_t>(s);
            h *= 1099511628211ULL;
        }
        return h;
    }
};

} // namespace SolidMesh
