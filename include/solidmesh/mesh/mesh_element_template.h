#pragma once
#include <array>
namespace SolidMesh
{
    enum class FaceType
    {
        UNSET,
        TRIANGLE,
        QUADRILATERAL
    };
    enum class CellType
    {
        UNSET,
        TET,
        HEX,
        PYRAMID,
        PRISM
    };
    namespace CellOrderTemplate
    {
        // Outward rings, with the same local numbering and row order as DynamicSolidMesh.
        inline constexpr std::array<std::array<int, 3>, 4> tet_faces = {{{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}}};
        inline constexpr std::array<std::array<int, 4>, 6> hex_faces = {{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}}};
        inline constexpr std::array<std::array<int, 4>, 5> pyramid_faces = {{{0, 3, 2, 1}, {0, 1, 4, -1}, {1, 2, 4, -1}, {2, 3, 4, -1}, {3, 0, 4, -1}}};
        // SolidMesh extension retaining its existing VTK_WEDGE support.
        inline constexpr std::array<std::array<int, 4>, 5> prism_faces = {{{0, 2, 1, -1}, {3, 4, 5, -1}, {0, 1, 4, 3}, {1, 2, 5, 4}, {2, 0, 3, 5}}};
    }
}
