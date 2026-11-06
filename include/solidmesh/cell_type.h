#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>

namespace SolidMesh
{
    enum class FaceType
    {
        UNSET,
        TRIANGLE,
        QUAD
    };

    enum class CellType
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

        static constexpr int max_cell_face_count = 6;   // 支持的单元内最大面数
        static constexpr int max_cell_vertex_count = 8; // 支持的单元内最大顶点数
        static constexpr int max_face_vertex_count = 4; // 支持的面内最大顶点数

        // ================================================================================
        // 采用VTK的单元内顶点排序，
        // 注意：增加模版需要同步修改face_orders中的switch分支
        // ================================================================================
        using FaceOrder = std::array<int, max_face_vertex_count>;

        // VTK_TETRA
        inline static constexpr auto tet_faces = std::to_array<FaceOrder>({{0, 1, 3, -1},
                                                                           {1, 2, 3, -1},
                                                                           {2, 0, 3, -1},
                                                                           {0, 2, 1, -1}});

        // VTK_HEXAHEDRON
        inline static constexpr auto hex_faces = std::to_array<FaceOrder>({
            {0, 4, 7, 3}, // 0
            {1, 2, 6, 5}, // 1
            {0, 1, 5, 4}, // 2
            {3, 7, 6, 2}, // 3
            {0, 3, 2, 1}, // 4
            {4, 5, 6, 7}  // 5
        });

        // VTK_PYRAMID
        inline static constexpr auto pyramid_faces = std::to_array<FaceOrder>({
            {0, 3, 2, 1},  // 0
            {0, 1, 4, -1}, // 1
            {1, 2, 4, -1}, // 2
            {2, 3, 4, -1}, // 3
            {3, 0, 4, -1}  // 4
        });

        /* TODO: VTK_WEDGE
            { 0, 2, 1, -1, -1 }, // 0
            { 3, 4, 5, -1, -1 }, // 1
            { 0, 1, 4, 3, -1 },  // 2
            { 1, 2, 5, 4, -1 },  // 3
            { 2, 0, 3, 5, -1 },  // 4
        */

        static constexpr std::size_t cell_face_count(CellType type)
        {
            return face_orders(type).size();
        }

        static constexpr std::size_t cell_vertex_count(CellType type)
        {
            // 局部顶点从 0 连续编号，末尾的 -1 填充项不影响最大值。
            int max_vertex = -1;
            for (const FaceOrder &face : face_orders(type))
                for (int vertex : face)
                    max_vertex = std::max(max_vertex, vertex);
            return static_cast<std::size_t>(max_vertex + 1);
        }

        // 按面的绕序返回局部顶点编号，区间只包含有效顶点。
        static constexpr std::span<const int> cell_face_order(CellType type, std::size_t face_index)
        {
            const auto faces = face_orders(type);
            if (face_index >= faces.size())
                throw std::runtime_error("Face index out of range");

            const FaceOrder &face = faces[face_index];
            return {face.begin(), std::find(face.begin(), face.end(), -1)};
        }

        static constexpr FaceType cell_face_type(CellType type, std::size_t face_index)
        {
            switch (cell_face_order(type, face_index).size())
            {
            case 3:
                return FaceType::TRIANGLE;
            case 4:
                return FaceType::QUAD;
            default:
                throw std::runtime_error("Unsupported face vertex count");
            }
        }

    private:
        static constexpr std::span<const FaceOrder> face_orders(CellType type)
        {
            switch (type)
            {
            case CellType::TET:
                return tet_faces;
            case CellType::HEX:
                return hex_faces;
            case CellType::PYRAMID:
                return pyramid_faces;
            default:
                throw std::runtime_error("Unsupported cell type");
            }
        }
    };


}
