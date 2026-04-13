#pragma once

#include "MeshEntities.h"
#include <cstdint>

namespace SolidMesh {

// Per-face descriptor: how many vertices, and which local vertex indices
// (indices into Cell::verts[]) make up this face.
struct FaceDesc {
    uint8_t vcount;           // 3 = triangle, 4 = quad
    uint8_t local_verts[4];   // indices into Cell::verts[]; unused = 0
};

// ---- Tetrahedron ----
// Vertices: {v0, v1, v2, v3}
// Face winding: CCW as seen from outside the tet (outward normal by right-hand rule).
// Each face is opposite the vertex with the same index.
static constexpr FaceDesc TET_FACES[4] = {
    {3, {1, 2, 3, 0}},  // face 0: opposite v0, vertices v1-v2-v3
    {3, {0, 3, 2, 0}},  // face 1: opposite v1, vertices v0-v3-v2
    {3, {0, 1, 3, 0}},  // face 2: opposite v2, vertices v0-v1-v3
    {3, {0, 2, 1, 0}},  // face 3: opposite v3, vertices v0-v2-v1
};
static constexpr uint32_t TET_FACE_COUNT   = 4;
static constexpr uint32_t TET_VERTEX_COUNT = 4;

// ---- Hexahedron ----
// Vertices: bottom ring {v0,v1,v2,v3} CCW from outside, top ring {v4,v5,v6,v7}
// v4 above v0, v5 above v1, v6 above v2, v7 above v3.
static constexpr FaceDesc HEX_FACES[6] = {
    {4, {0, 3, 2, 1}},  // face 0: bottom  (v0-v3-v2-v1, inward normal = up)
    {4, {4, 5, 6, 7}},  // face 1: top     (v4-v5-v6-v7)
    {4, {0, 1, 5, 4}},  // face 2: front   (v0-v1-v5-v4)
    {4, {1, 2, 6, 5}},  // face 3: right   (v1-v2-v6-v5)
    {4, {2, 3, 7, 6}},  // face 4: back    (v2-v3-v7-v6)
    {4, {3, 0, 4, 7}},  // face 5: left    (v3-v0-v4-v7)
};
static constexpr uint32_t HEX_FACE_COUNT   = 6;
static constexpr uint32_t HEX_VERTEX_COUNT = 8;

// ---- Pyramid ----
// Vertices: base quad {v0,v1,v2,v3} CCW from outside, apex v4.
static constexpr FaceDesc PYR_FACES[5] = {
    {4, {0, 3, 2, 1}},  // face 0: base quad (v0-v3-v2-v1)
    {3, {0, 1, 4, 0}},  // face 1: front tri (v0-v1-v4)
    {3, {1, 2, 4, 0}},  // face 2: right tri (v1-v2-v4)
    {3, {2, 3, 4, 0}},  // face 3: back  tri (v2-v3-v4)
    {3, {3, 0, 4, 0}},  // face 4: left  tri (v3-v0-v4)
};
static constexpr uint32_t PYR_FACE_COUNT   = 5;
static constexpr uint32_t PYR_VERTEX_COUNT = 5;

// ---- Prism (triangular prism) ----
// Vertices: bottom tri {v0,v1,v2} CCW from outside, top tri {v3,v4,v5}
// v3 above v0, v4 above v1, v5 above v2.
static constexpr FaceDesc PRISM_FACES[5] = {
    {3, {0, 2, 1, 0}},  // face 0: bottom tri (v0-v2-v1, normal points down)
    {3, {3, 4, 5, 0}},  // face 1: top    tri (v3-v4-v5, normal points up)
    {4, {0, 1, 4, 3}},  // face 2: quad side  (v0-v1-v4-v3)
    {4, {1, 2, 5, 4}},  // face 3: quad side  (v1-v2-v5-v4)
    {4, {2, 0, 3, 5}},  // face 4: quad side  (v2-v0-v3-v5)
};
static constexpr uint32_t PRISM_FACE_COUNT   = 5;
static constexpr uint32_t PRISM_VERTEX_COUNT = 6;

// ---- Lookup helpers ----

inline uint32_t face_count_for(CellType t) {
    switch (t) {
        case CellType::Tet:     return TET_FACE_COUNT;
        case CellType::Hex:     return HEX_FACE_COUNT;
        case CellType::Pyramid: return PYR_FACE_COUNT;
        case CellType::Prism:   return PRISM_FACE_COUNT;
    }
    return 0;
}

inline uint32_t vertex_count_for(CellType t) {
    switch (t) {
        case CellType::Tet:     return TET_VERTEX_COUNT;
        case CellType::Hex:     return HEX_VERTEX_COUNT;
        case CellType::Pyramid: return PYR_VERTEX_COUNT;
        case CellType::Prism:   return PRISM_VERTEX_COUNT;
    }
    return 0;
}

inline const FaceDesc* face_table_for(CellType t) {
    switch (t) {
        case CellType::Tet:     return TET_FACES;
        case CellType::Hex:     return HEX_FACES;
        case CellType::Pyramid: return PYR_FACES;
        case CellType::Prism:   return PRISM_FACES;
    }
    return nullptr;
}

} // namespace SolidMesh
