#pragma once
#include "mesh_element.h"
#include <span>

namespace SolidMesh
{

    // ============================================================
    // mesh本体类
    // ============================================================

    class PolyhedraMesh
    {
        using VertexStorage   = ElementStorage<Vertex>;
        using DedgeStorage    = ElementStorage<Dedge>;
        using HalffaceStorage = ElementStorage<Halfface>;
        using CellStorage     = ElementStorage<Cell>;

    public:
        PolyhedraMesh() = default;
        ~PolyhedraMesh() = default;
        PolyhedraMesh(const PolyhedraMesh&) = default;
        PolyhedraMesh& operator=(const PolyhedraMesh& other);
        PolyhedraMesh(PolyhedraMesh&& other);
        PolyhedraMesh& operator=(PolyhedraMesh&& other);

    public:
        using SizeType = VertexStorage::size_type;

        // 基本信息
        SizeType num_vertices() const;
        SizeType num_dedges() const;
        SizeType num_halffaces() const;
        SizeType num_cells() const;

        VertexHandle add_isolated_vertex(Real x, Real y, Real z);
        // 顶点按 CellTopology 的局部编号排列。类型不支持、数量不符、handle 无效或重复时返回空 handle。
        // 调用方负责保证增删后的网格仍为共形流形。内存分配异常时不保证回滚。
        CellHandle add_cell(CellType type, std::span<const VertexHandle> vertices);
        bool delete_isolated_vertex(VertexHandle handle);
        bool delete_cell(CellHandle handle); // 清理无用边和半面，保留孤立顶点。

        // range for 遍历；增删前应先收集 handle 快照。
        HandleRange<VertexStorage, VertexHandle> vertices() const noexcept;
        HandleRange<DedgeStorage, DedgeHandle> dedges() const noexcept;
        HandleRange<HalffaceStorage, HalffaceHandle> halffaces() const noexcept;
        HandleRange<CellStorage, CellHandle> cells() const noexcept;

        // 元素访问
        const Vertex *vertex(VertexHandle handle) const;
        const Dedge *dedge(DedgeHandle handle) const;
        const Halfface *halfface(HalffaceHandle handle) const;
        const Cell *cell(CellHandle handle) const;

        // 复制
        PolyhedraMesh copy() const;

        // 修改
        bool set_vertex_position(VertexHandle handle, Real x, Real y, Real z);

    public:
        // 邻接 1-ring
        std::vector<VertexHandle> vertex_vertices(VertexHandle handle) const; // vertex

        std::vector<CellHandle> edge_cells(DedgeHandle handle) const; // edge
        // 正向模板下，从 target 看向 source 为逆时针。边界返回开链，内部返回环（不重复首项）。
        std::vector<CellHandle> edge_cells_ccw(DedgeHandle handle) const;

        std::vector<CellHandle> cell_cells(CellHandle handle) const; // cell

    private:
        DedgeHandle find_dedge(VertexHandle source, VertexHandle target) const;
        HalffaceHandle find_opposite(const std::vector<VertexHandle>& face) const;
        void swap(PolyhedraMesh& other) noexcept;

    private:
        VertexStorage vertices_;
        DedgeStorage dedges_;
        HalffaceStorage halffaces_;
        CellStorage cells_;
    };
}
