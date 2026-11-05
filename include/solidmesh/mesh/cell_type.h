#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>

namespace SolidMesh
{
    enum class FaceType : uint8_t
    {
        UNSET,
        TRIANGLE,
        QUAD
    };

    enum class CellType : uint8_t
    {
        UNSET,
        TET,
        HEX,
        PYRAMID
    };

    class CellTopology
    {
    public:
        CellTopology() = delete;

        static constexpr int template_max_cell_face_count = 6;   // 支持的单元内最大面数
        static constexpr int template_max_cell_vertex_count = 8; // 支持的单元内最大顶点数
        static constexpr int template_max_face_vertex_count = 4; // 支持的面内最大顶点数

        // ================================================================================
        // 采用VTK的单元内顶点排序
        // ================================================================================

        // VTK_TETRA
        inline static constexpr std::array<std::array<int, 3>, 4> tet_faces = {{
            {0, 1, 3}, // 0
            {1, 2, 3}, // 1
            {2, 0, 3}, // 2
            {0, 2, 1}  // 3
        }};

        // VTK_HEXAHEDRON
        inline static constexpr std::array<std::array<int, 4>, 6> hex_faces = {{
            {0, 4, 7, 3}, // 0
            {1, 2, 6, 5}, // 1
            {0, 1, 5, 4}, // 2
            {3, 7, 6, 2}, // 3
            {0, 3, 2, 1}, // 4
            {4, 5, 6, 7}  // 5
        }};

        // VTK_PYRAMID
        inline static constexpr std::array<std::array<int, 4>, 5> pyramid_faces = {{
            {0, 3, 2, 1},  // 0
            {0, 1, 4, -1}, // 1
            {1, 2, 4, -1}, // 2
            {2, 3, 4, -1}, // 3
            {3, 0, 4, -1}  // 4
        }};

        /* TODO: VTK_WEDGE
            { 0, 2, 1, -1, -1 }, // 0
            { 3, 4, 5, -1, -1 }, // 1
            { 0, 1, 4, 3, -1 },  // 2
            { 1, 2, 5, 4, -1 },  // 3
            { 2, 0, 3, 5, -1 },  // 4
        */

    public:
        inline static std::size_t cell_face_count(CellType type)
        {
            switch (type)
            {
            case CellType::TET:     return tet_faces.size();
            case CellType::HEX:     return hex_faces.size();
            case CellType::PYRAMID: return pyramid_faces.size();
            default: throw std::runtime_error("Unsupported cell type");
            }
        }

        inline static std::span<const int> cell_face_order(CellType type, std::size_t face_index)
        {
            switch (type)
            {
            case CellType::TET:
                if (face_index >= tet_faces.size())
                    throw std::runtime_error("Face index out of range for TET");
                return tet_faces[face_index];
            case CellType::HEX:
                if (face_index >= hex_faces.size())
                    throw std::runtime_error("Face index out of range for HEX");
                return hex_faces[face_index];
            case CellType::PYRAMID:
                if (face_index >= pyramid_faces.size())
                    throw std::runtime_error("Face index out of range for PYRAMID");
                return pyramid_faces[face_index];
            default:
                throw std::runtime_error("Unsupported cell type");
            }
        }

    };

}
