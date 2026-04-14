#pragma once
#include <array>
#include <cstdint>

namespace SolidMesh {

enum class CellType : uint8_t {
    Tet,      // 4 verts, 4 tri faces
    Hex,      // 8 verts, 6 quad faces
    Prism,    // 6 verts, 2 tri + 3 quad faces
    Pyramid   // 5 verts, 1 quad + 4 tri faces
};

// Describes one face of a cell in terms of local vertex indices.
struct FaceLocalDesc {
    int size;        // number of vertices (3 or 4)
    int vids[4];     // local vertex indices; unused slots are -1
};

struct CellTopologyTraits {
    int           num_vertices;
    int           num_faces;
    FaceLocalDesc faces[6];
};

// Face vertex orderings follow the outward-normal right-hand-rule convention.
// These match common VTK/OpenVolumeMesh conventions.
inline const CellTopologyTraits& get_cell_topology(CellType type) {
    // Tet: verts 0-3
    //   face 0: 0,2,1  (base, opposite v3)
    //   face 1: 0,1,3
    //   face 2: 1,2,3
    //   face 3: 0,3,2
    static const CellTopologyTraits tet = {
        4, 4,
        {
            {3, {0, 2, 1, -1}},
            {3, {0, 1, 3, -1}},
            {3, {1, 2, 3, -1}},
            {3, {0, 3, 2, -1}},
        }
    };

    // Hex: verts 0-7 (bottom 0-3, top 4-7, same winding)
    //   face 0: 0,3,2,1  (bottom, outward = downward)
    //   face 1: 4,5,6,7  (top)
    //   face 2: 0,1,5,4  (front)
    //   face 3: 1,2,6,5  (right)
    //   face 4: 2,3,7,6  (back)
    //   face 5: 3,0,4,7  (left)
    static const CellTopologyTraits hex = {
        8, 6,
        {
            {4, {0, 3, 2, 1}},
            {4, {4, 5, 6, 7}},
            {4, {0, 1, 5, 4}},
            {4, {1, 2, 6, 5}},
            {4, {2, 3, 7, 6}},
            {4, {3, 0, 4, 7}},
        }
    };

    // Prism: verts 0-5 (bottom tri 0-2, top tri 3-5)
    //   face 0: 0,2,1    (bottom tri, outward = downward)
    //   face 1: 3,4,5    (top tri)
    //   face 2: 0,1,4,3  (front quad)
    //   face 3: 1,2,5,4  (right quad)
    //   face 4: 2,0,3,5  (left quad)
    static const CellTopologyTraits prism = {
        6, 5,
        {
            {3, {0, 2, 1, -1}},
            {3, {3, 4, 5, -1}},
            {4, {0, 1, 4,  3}},
            {4, {1, 2, 5,  4}},
            {4, {2, 0, 3,  5}},
            {0, {-1,-1,-1,-1}},  // unused
        }
    };

    // Pyramid: verts 0-4 (base quad 0-3, apex 4)
    //   face 0: 0,3,2,1  (base quad, outward = downward)
    //   face 1: 0,1,4    (front tri)
    //   face 2: 1,2,4    (right tri)
    //   face 3: 2,3,4    (back tri)
    //   face 4: 3,0,4    (left tri)
    static const CellTopologyTraits pyramid = {
        5, 5,
        {
            {4, {0, 3, 2,  1}},
            {3, {0, 1, 4, -1}},
            {3, {1, 2, 4, -1}},
            {3, {2, 3, 4, -1}},
            {3, {3, 0, 4, -1}},
            {0, {-1,-1,-1,-1}},  // unused
        }
    };

    switch (type) {
        case CellType::Tet:     return tet;
        case CellType::Hex:     return hex;
        case CellType::Prism:   return prism;
        case CellType::Pyramid: return pyramid;
    }
    return tet; // unreachable
}

} // namespace SolidMesh
