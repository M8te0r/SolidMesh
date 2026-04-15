#pragma once
#include <cstdint>
#include <vector>
#include "solidmesh/mesh/handles.h"

namespace SolidMesh {

// Canonical, unoriented key for a polygonal face.
// Two halffaces sharing the same geometric face produce the same FaceKey
// regardless of orientation or starting vertex.
//
// Uses a fixed-size array (max 4 vertices) to avoid heap allocation.
struct FaceKey {
    uint8_t  n = 0;
    uint32_t slots[4] = {};

    bool operator==(const FaceKey& o) const noexcept {
        if (n != o.n) return false;
        for (int i = 0; i < n; ++i)
            if (slots[i] != o.slots[i]) return false;
        return true;
    }
    bool operator!=(const FaceKey& o) const noexcept { return !(*this == o); }
};

inline FaceKey make_face_key(const std::vector<VertexID>& verts) {
    const int n = static_cast<int>(verts.size());

    bool best_flip  = false;
    int  best_start = 0;

    auto slot_at = [&](bool flip, int start, int i) -> uint32_t {
        return flip ? verts[(start - i + n) % n].slot
                    : verts[(start + i) % n].slot;
    };

    auto is_better = [&](bool flip, int start) -> bool {
        for (int i = 0; i < n; ++i) {
            uint32_t a = slot_at(flip, start, i);
            uint32_t b = slot_at(best_flip, best_start, i);
            if (a < b) return true;
            if (a > b) return false;
        }
        return false;
    };

    for (int s = 0; s < n; ++s) {
        if (is_better(false, s)) { best_flip = false; best_start = s; }
        if (is_better(true,  s)) { best_flip = true;  best_start = s; }
    }

    FaceKey key;
    key.n = static_cast<uint8_t>(n);
    for (int i = 0; i < n; ++i) {
        int idx = best_flip ? (best_start - i + n) % n : (best_start + i) % n;
        key.slots[i] = verts[idx].slot;
    }
    return key;
}

struct FaceKeyHash {
    size_t operator()(const FaceKey& k) const noexcept {
        // FNV-1a over the slot array
        size_t h = 14695981039346656037ULL;
        for (int i = 0; i < k.n; ++i) {
            h ^= static_cast<size_t>(k.slots[i]);
            h *= 1099511628211ULL;
        }
        return h;
    }
};

} // namespace SolidMesh
