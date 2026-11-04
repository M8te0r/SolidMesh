#pragma once

#include <cstddef>
#include <vector>

#include <solidmesh/mesh/mesh_element.h>

namespace SolidMesh
{
    class Mesh
    {
    public:
        Mesh() = default;

        // 输入采用 CellOrderTemplate 的局部编号，顶点和单元 ID 保持输入顺序。
        // 默认输入为合法的共形流形网格；无法构建时抛出 std::runtime_error。
        [[nodiscard]]
        static Mesh from_data(const std::vector<Vector3> &vertices,
                              const std::vector<std::vector<IndexType>> &cells,
                              const std::vector<CellType> &celltypes);

    public:

        // ---- 元素数量 -------------------------------------------------
        std::size_t num_vertices() const noexcept;
        std::size_t num_cells() const noexcept;
        std::size_t num_faces() const noexcept;
        std::size_t num_halffaces() const noexcept;
        std::size_t num_edges() const noexcept;
        std::size_t num_dedges() const noexcept;

        // ---- 查找 -------------------------------------------------
        // idx 必须有效；调试构建使用断言检查。引用借用 Mesh 内部存储。
        const Vertex &vertex(IndexType idx) const noexcept;
        const DirectedEdge &dedge(IndexType idx) const noexcept;
        const Edge &edge(IndexType idx) const noexcept;
        const HalfFace &halfface(IndexType idx) const noexcept;
        const Face &face(IndexType idx) const noexcept;
        const Cell &cell(IndexType idx) const noexcept;

        // ---- topology queries ----------------------------------

        // 以下邻接查询对无效 ID 返回空 vector。
        // 以 vi 为起点的所有有向边，不承诺几何顺序。
        std::vector<IndexType> vertex_dedges(IndexType vi) const;

        // 通过网格边相连的顶点；去重、不含自身，不承诺几何顺序。
        std::vector<IndexType> vertex_vertices(IndexType vi) const;

        // 有向边的 1-ring cells：从终点望向起点时为 CCW（右手法则）。
        // 内部边为不重复首项的闭环，以最小 cell ID 开始；边界边为完整开链。
        // 反向 dedge 反转顺序，闭环保持最小 cell ID 为首项。
        // 顺序在构建时按拓扑缓存，几何含义以正确的正向编号为前提。
        std::vector<IndexType> dedge_cells_ccw(IndexType dei) const;

        // 共享面的相邻单元；去重、不含自身或边界占位项，不承诺几何顺序。
        std::vector<IndexType> cell_cells(IndexType ci) const;

    private:
        // ---- 元素容器 ----------------------------------
        std::vector<Vertex> vertices_;
        std::vector<Cell> cells_;
        std::vector<Face> faces_;
        std::vector<HalfFace> halffaces_;
        std::vector<Edge> edges_;
        std::vector<DirectedEdge> dedges_;

        // ---- Compressed Sparse Row ----------------------------------
        std::vector<IndexType> vertex_dedges_;              // 所有顶点的出向 dedge ID 展平
        std::vector<std::size_t> vertex_dedge_offsets_{0};  // 顶点数 + 1 项，末项为邻接总长度
        std::vector<IndexType> dedge_cells_;                // 所有定向边的1-ring cells[]展平
        std::vector<std::size_t> dedge_cell_offsets_{0};    // 有向边数 + 1 项，末项为邻接总长度
    };
}
