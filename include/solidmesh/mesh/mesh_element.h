#pragma once
#include <array>
#include <cstdint>
#include <limits>

#include <solidmesh/math/vector3.h>
#include <solidmesh/mesh/cell_type.h>

namespace SolidMesh
{

    using IndexType = std::uint32_t;
    static constexpr IndexType invalid_id = std::numeric_limits<IndexType>::max();
    

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
        std::array<IndexType, 2> vertices{invalid_id, invalid_id}; // 起点、终点的vertex id

        IndexType opposite = invalid_id;                // 反向的DirectEdge
        IndexType edge = invalid_id;                    // 所属的Edge
    };

    struct Edge
    {
        IndexType de0 = invalid_id;
        IndexType de1 = invalid_id; // DirectEdge的id
    };

    struct HalfFace
    {
        std::array<IndexType, 4> dedges = {invalid_id, invalid_id, invalid_id, invalid_id};   // 有序的DirectedEdge环

        IndexType opposite = invalid_id; // 反面halfface的id

        IndexType face = invalid_id;    // 所属的face
        IndexType cell = invalid_id;    // 所属的cell
    };

    struct Face
    {
        FaceType type = FaceType::UNSET;
        std::array<IndexType, 2> halffaces{invalid_id, invalid_id}; // halfface的id
    };

    struct Cell
    {
        CellType type = CellType::UNSET;
        std::array<IndexType, 8> vertices {invalid_id, invalid_id, invalid_id, invalid_id, invalid_id, invalid_id, invalid_id, invalid_id};
        std::array<IndexType, 6> halffaces = {invalid_id, invalid_id, invalid_id, invalid_id, invalid_id, invalid_id};
    };
}
