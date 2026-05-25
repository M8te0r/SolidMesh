#pragma once
#include <initializer_list>
#include <cstdint>

namespace SolidMesh
{
    enum class CellType : uint8_t
    {
        Invalid,
        Tet,
        Hex,
        Prism,
        Pyramid
    };

    static constexpr CellType vtk_cell_type(int type_num)
    {
        switch (type_num)
        {
        case 10:
            return CellType::Tet;
        case 12:
            return CellType::Hex;
        case 13:
            return CellType::Prism;
        case 14:
            return CellType::Pyramid;
        default:
            return CellType::Invalid;
        }
    }

    // 定义
    static constexpr int cell_traits_null = -1;
    static constexpr int cell_traits_max_cell_face_count = 6;
    static constexpr int cell_traits_max_cell_vertex_count = 8;
    static constexpr int cell_traits_max_face_vertex_count = 4;

    // 规整平铺的拓扑结构
    struct CellTopoPadded
    {
        int face_vertex_count[cell_traits_max_cell_face_count]{};
        int local_faces[cell_traits_max_cell_face_count][cell_traits_max_face_vertex_count]{};
    };

    // 编译期自动对齐
    static constexpr CellTopoPadded padding_topology(std::initializer_list<std::initializer_list<int>> init_faces)
    {
        CellTopoPadded padded{};
        // 默认填满 null
        for (int i = 0; i < cell_traits_max_cell_face_count; ++i)
        {
            padded.face_vertex_count[i] = 0;
            for (int j = 0; j < cell_traits_max_face_vertex_count; ++j)
            {
                padded.local_faces[i][j] = cell_traits_null;
            }
        }
        // 自动解析拓扑
        size_t f_idx = 0;
        for (const auto &face : init_faces)
        {
            if (f_idx >= cell_traits_max_cell_face_count)
                break;
            padded.face_vertex_count[f_idx] = static_cast<int>(face.size());
            size_t v_idx = 0;
            for (int v : face)
            {
                if (v_idx >= cell_traits_max_face_vertex_count)
                    break;
                padded.local_faces[f_idx][v_idx] = v;
                v_idx++;
            }
            f_idx++;
        }
        return padded;
    }

    // ========================================================
    // 2. 完美的 Traits 核心（无继承，零干扰，只需填核心字段）
    // ========================================================
    template <CellType Type>
    struct CellTraits;

    // --- 四面体 ---
    template <>
    struct CellTraits<CellType::Tet>
    {
        static constexpr int vertex_count = 4;
        static constexpr int face_count = 4;
        static constexpr CellTopoPadded topo = padding_topology(
            {{0, 1, 3},
             {0, 3, 2},
             {0, 2, 1},
             {1, 2, 3}});
    };

    // --- 六面体 ---
    template <>
    struct CellTraits<CellType::Hex>
    {
        static constexpr int vertex_count = 8;
        static constexpr int face_count = 6;
        static constexpr CellTopoPadded topo = padding_topology(
            {{0, 3, 2, 1},
             {4, 5, 6, 7},
             {0, 1, 5, 4},
             {1, 2, 6, 5},
             {2, 3, 7, 6},
             {3, 0, 4, 7}});
    };

    // --- 三棱柱 ---
    template <>
    struct CellTraits<CellType::Prism>
    {
        static constexpr int vertex_count = 6;
        static constexpr int face_count = 5;
        static constexpr CellTopoPadded topo = padding_topology(
            {{0, 2, 1},
             {3, 4, 5},
             {0, 1, 4, 3},
             {1, 2, 5, 4},
             {2, 0, 3, 5}});
    };

    // --- 金字塔 ---
    template <>
    struct CellTraits<CellType::Pyramid>
    {
        static constexpr int vertex_count = 5;
        static constexpr int face_count = 5;
        static constexpr CellTopoPadded topo = padding_topology(
            {{0, 3, 2, 1},
             {0, 1, 4},
             {1, 2, 4},
             {2, 3, 4},
             {3, 0, 4}});
    };

    inline int cell_vertex_count(CellType type)
    {
        switch (type)
        {
        case CellType::Tet:
            return CellTraits<CellType::Tet>::vertex_count;
        case CellType::Hex:
            return CellTraits<CellType::Hex>::vertex_count;
        case CellType::Prism:
            return CellTraits<CellType::Prism>::vertex_count;
        case CellType::Pyramid:
            return CellTraits<CellType::Pyramid>::vertex_count;
        default:
            return 0;
        }
    }

    inline int cell_face_count(CellType type)
    {
        switch (type)
        {
        case CellType::Tet:
            return CellTraits<CellType::Tet>::face_count;
        case CellType::Hex:
            return CellTraits<CellType::Hex>::face_count;
        case CellType::Prism:
            return CellTraits<CellType::Prism>::face_count;
        case CellType::Pyramid:
            return CellTraits<CellType::Pyramid>::face_count;
        default:
            return 0;
        }
    }

    inline const CellTopoPadded &cell_topo(CellType type)
    {
        switch (type)
        {
        case CellType::Tet:
            return CellTraits<CellType::Tet>::topo;
        case CellType::Hex:
            return CellTraits<CellType::Hex>::topo;
        case CellType::Prism:
            return CellTraits<CellType::Prism>::topo;
        case CellType::Pyramid:
            return CellTraits<CellType::Pyramid>::topo;
        default:
            return CellTraits<CellType::Tet>::topo;
        }
    }

}

// HalfFaceID = (FaceID << 1) | Orientation
