#pragma once
#include <limits>

#include <solidmesh/math/vector3.h>
#include <solidmesh/mesh/cell_type.h>

namespace SolidMesh
{

    using ElementID = uint32_t;
    static constexpr ElementID invalid_id = std::numeric_limits<ElementID>::max();
    

    struct Vertex;
    struct DirectedEdge;
    struct Edge;
    struct HalfFace;
    struct Face;
    struct Cell;

    struct Vertex
    {
        Vector3 point = Vector3::undefined();
    };

    struct DirectedEdge
    {
        std::array<ElementID, 2> vertices{};            // 起点、终点的vertex id

        ElementID opposite = invalid_id;                // 反向的DirectEdge
        ElementID edge = invalid_id;                    // 所属的Edge
    };

    struct Edge
    {
        ElementID de0 = invalid_id;
        ElementID de1 = invalid_id; // DirectEdge的id
    };

    struct HalfFace
    {
        std::array<ElementID, 4> dedges = {};   // 有序的DirectedEdge环

        ElementID opposite = invalid_id; // 反面halfface的id

        ElementID face = invalid_id;    // 所属的face
        ElementID cell = invalid_id;    // 所属的cell
        int cell_local_face = -1;
    };

    struct Face
    {
        FaceType type = FaceType::Invalid;
        std::array<ElementID, 2> halffaces{}; // halfface的id
    };

    struct Cell
    {
        CellType type = CellType::Invalid;
        std::array<ElementID, 8> vertices {};
        std::array<ElementID, 6> halffaces = {};
    };
}