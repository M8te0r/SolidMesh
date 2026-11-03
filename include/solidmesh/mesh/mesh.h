#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <solidmesh/mesh/mesh_element.h>

namespace SolidMesh
{
    class Mesh
    {
        using IndexType = uint32_t;
    public:
        Mesh() = default;
        ~Mesh() = default;

    public:
        // 构造拓扑
        void build_topology(const std::vector<std::vector<double>> &points,
                            const std::vector<std::vector<uint32_t>> &cells,
                            const std::vector<CellType> &celltypes);

        // ---- 元素数量 -------------------------------------------------
        size_t num_vertices() const noexcept;
        size_t num_cells() const noexcept;
        size_t num_faces() const noexcept;
        size_t num_halffaces() const noexcept;
        size_t num_edges() const noexcept;
        size_t num_dedges() const noexcept;

        // ---- 查找 -------------------------------------------------
        const Vertex &vertex(IndexType idx) const noexcept;
        const DirectedEdge &dedge(IndexType idx) const noexcept;
        const Edge &edge(IndexType idx) const noexcept;
        const HalfFace &halfface(IndexType idx) const noexcept;
        const Face &face(IndexType idx) const noexcept;
        const Cell &cell(IndexType idx) const noexcept;

        // ---- topology queries ----------------------------------

        // 顶点的1-ring顶点
        std::vector<IndexType> vertex_vertices(IndexType vi) const;

        // 有向边的 1-ring cells：从终点望向起点时为 CCW（右手法则）。
        std::vector<IndexType> edge_cells_ccw(IndexType dei) const;

        // 有向面上的顶点（严格 CCW 顺序）
        std::vector<IndexType> halfface_vertices(IndexType hfi) const;

        // Cell 邻接的周边 cells
        std::vector<IndexType> cell_cells(IndexType ci) const;

        // Cell上的所有定向面
        std::vector<IndexType> cell_halffaces(IndexType ci) const;

        // Cell 的vertices
        std::vector<IndexType> cell_vertices(IndexType ci) const;

    private:
        // ---- 元素容器 ----------------------------------
        std::vector<Vertex> vertices_;
        std::vector<Cell> cells_;
        std::vector<Face> faces_;
        std::vector<HalfFace> halffaces_;
        std::vector<Edge> edges_;
        std::vector<DirectedEdge> dedges_;

        // ---- Compressed Sparse Row ----------------------------------
        std::vector<IndexType> vertex_vertex_offsets_;
        std::vector<IndexType> vertex_vertices_;
        std::vector<IndexType> dedge_cell_offsets_;
        std::vector<IndexType> dedge_cells_;
    };
}
