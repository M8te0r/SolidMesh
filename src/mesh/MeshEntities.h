#pragma once

#include "Handles.h"
#include "linear-algebra/vector3.h"
#include <array>
#include <cstdint>

namespace SolidMesh {

// ---- Cell type ----
enum class CellType : uint8_t {
    Tet     = 0,  // 4 triangular faces, 4 vertices
    Hex     = 1,  // 6 quad faces,       8 vertices
    Pyramid = 2,  // 1 quad + 4 tri,     5 vertices
    Prism   = 3,  // 2 tri + 3 quad,     6 vertices
};

// ---- Vertex ----
// ~56 bytes, fits in one cache line.
struct Vertex {
    Vector3   position;
    Vector3   normal;    // zero-initialized; optional
    CellID   one_cell;  // any one incident cell (adjacency seed); INVALID_ID if isolated
    uint32_t _pad;      // for memory alignment
};

// ---- Face ----
// Stores both half-face payloads in one record.
// half-face 0 = make_hf(id, 0), owned by cell[0]
// half-face 1 = make_hf(id, 1), owned by cell[1]  (INVALID_ID if boundary)
// Winding: verts[] are in CCW order as seen from outside cell[0].
//          cell[1] sees them in CW order (same physical face, opposite orientation).
struct Face {
    std::array<VertexID, 4> verts;  // [0..2] for tri, [0..3] for quad; unused = INVALID_ID
    uint8_t  vert_count;            // 3 (tri) or 4 (quad)
    uint8_t  _pad[3];               // for memory alignment
    CellID   cell[2];               // owning cells; INVALID_ID = boundary side
};

// ---- Cell ----
// ~60 bytes, fits in one cache line.
struct Cell {
    CellType type;
    uint8_t  _pad[3];       // for memory alignment
    // Half-face IDs in canonical order per CellTopology tables.
    // Unused slots = INVALID_ID.
    std::array<HalfFaceID, 6> hfaces;
    // Vertex IDs in canonical order per CellTopology tables.
    // Unused slots = INVALID_ID.
    std::array<VertexID, 8> verts;
};

} // namespace SolidMesh
