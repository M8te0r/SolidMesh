#pragma once
#include <initializer_list>
#include <cstdint>
#include <array>

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

    namespace CellOrderTemplate
    {
        static constexpr int template_max_cell_face_count = 6;   // 支持的单元内最大面数
        static constexpr int template_max_cell_vertex_count = 8; // 支持的单元内最大顶点数
        static constexpr int template_max_face_vertex_count = 4; // 支持的面内最大顶点数

        // ================================================================================
        // 采用VTK的单元内顶点排序
        // ================================================================================

        /* VTK_TETRA
            { 0, 1, 3, -1 }, // 0
            { 1, 2, 3, -1 }, // 1
            { 2, 0, 3, -1 }, // 2
            { 0, 2, 1, -1 }, // 3
        */

        inline static constexpr std::array<std::array<int, 3>, 4> tet_faces = {{
            {0, 1, 3}, // 0
            {1, 2, 3}, // 1
            {2, 0, 3}, // 2
            {0, 2, 1}  // 3
        }};

        /* VTK_HEXAHEDRON
            { 0, 4, 7, 3, -1 }, // 0
            { 1, 2, 6, 5, -1 }, // 1
            { 0, 1, 5, 4, -1 }, // 2
            { 3, 7, 6, 2, -1 }, // 3
            { 0, 3, 2, 1, -1 }, // 4
            { 4, 5, 6, 7, -1 }, // 5
        */

        inline static constexpr std::array<std::array<int, 4>, 6> hex_faces = {{
            {0, 4, 7, 3}, // 0
            {1, 2, 6, 5}, // 1
            {0, 1, 5, 4}, // 2
            {3, 7, 6, 2}, // 3
            {0, 3, 2, 1}, // 4
            {4, 5, 6, 7}  // 5
        }};

        /* VTK_PYRAMID
            { 0, 3, 2, 1, -1 },  // 0
            { 0, 1, 4, -1, -1 }, // 1
            { 1, 2, 4, -1, -1 }, // 2
            { 2, 3, 4, -1, -1 }, // 3
            { 3, 0, 4, -1, -1 }, // 4
        */

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
    };

}
