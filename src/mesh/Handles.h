#pragma once

#include <cstdint>

namespace SolidMesh {

// ---- ID typedefs ----
using VertexID   = uint32_t;
using FaceID     = uint32_t;
using HalfFaceID = uint32_t;
using CellID     = uint32_t;

// Packed (min_v, max_v) edge key — not a SlotMap ID
using EdgeKey = uint64_t;

constexpr uint32_t INVALID_ID = ~uint32_t(0);

// ---- HalfFaceID encoding ----
// HalfFaceID = FaceID * 2 + side   (side = 0 or 1)
// cell[side] is the owning cell of that half-face.
// Winding: side-0 half-face has CCW vertices as seen from outside cell[0].

inline FaceID     face_of(HalfFaceID hf)         { return hf >> 1; }
inline int        side_of(HalfFaceID hf)          { return static_cast<int>(hf & 1u); }
inline HalfFaceID make_hf(FaceID f, int side)     { return (f << 1) | static_cast<uint32_t>(side); }
inline HalfFaceID opposite_hf(HalfFaceID hf)     { return hf ^ 1u; }

// ---- EdgeKey helpers ----
inline EdgeKey make_edge_key(VertexID a, VertexID b) {
    if (a > b) { VertexID t = a; a = b; b = t; }
    return (static_cast<EdgeKey>(a) << 32) | static_cast<EdgeKey>(b);
}

} // namespace SolidMesh
