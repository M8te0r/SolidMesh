#pragma once
#include <vector>
#include <cstdint>
#include "solidmesh/math/vector3.h"
#include "solidmesh/mesh/handles.h"
#include "solidmesh/mesh/cell_topology.h"

namespace SolidMesh {

// Flags shared by all entity types
enum EntityFlags : uint32_t {
    FLAG_DELETED  = 1u << 0,
    FLAG_BOUNDARY = 1u << 1,   // cached; may be stale after edits
    FLAG_USER0    = 1u << 16,  // free for application use
    FLAG_USER1    = 1u << 17,
    FLAG_USER2    = 1u << 18,
    FLAG_USER3    = 1u << 19,
};

struct Vertex {
    Vector3     position;
    HalfFaceID  one_halfface;   // seed for incident-halfface traversal
    uint32_t    flags = 0;
};

// Internal entity — not directly exposed in the public API.
// Owns the canonical (unoriented) vertex ring for a topological face.
// Paired with at most two HalfFaces (one per incident cell).
struct Face {
    std::vector<VertexID> vertices;  // canonical ring (lexicographically minimal rotation)
    HalfFaceID            hf[2];     // hf[1] invalid => boundary face
    uint32_t              flags = 0;

    bool is_boundary() const noexcept { return !hf[1].is_valid(); }
};

// A directed view of a Face from one incident Cell.
// Stores the vertex ring as seen from the cell (oriented outward).
struct HalfFace {
    FaceID                face;
    CellID                cell;
    uint8_t               local_face_index = 0;
    std::vector<VertexID> vertices;   // oriented ring as seen from the cell
    uint32_t              flags = 0;
};

struct Cell {
    CellType                type;
    std::vector<VertexID>   vertices;   // matches CellTopologyTraits ordering
    std::vector<HalfFaceID> halffaces;  // halffaces[i] corresponds to local face i
    uint32_t                flags = 0;
};

} // namespace SolidMesh
