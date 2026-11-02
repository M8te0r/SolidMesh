#pragma once
#include "mesh_element.h"
#include <cstddef>
namespace SolidMesh {
class Mesh {
    using VertexStorage = ElementStorage<Vertex>;
    using DedgeStorage = ElementStorage<Dedge>;
    using HalffaceStorage = ElementStorage<Halfface>;
    using CellStorage = ElementStorage<Cell>;
public:
    Mesh() = default;
    ~Mesh() = default;
    Mesh(const Mesh&) = default;
    Mesh& operator=(const Mesh& other);
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;
    using SizeType = std::size_t;
    SizeType num_vertices() const noexcept;
    SizeType num_dedges() const noexcept;
    SizeType num_halffaces() const noexcept;
    SizeType num_cells() const noexcept;
    VertexHandle add_isolated_vertex(Real x, Real y, Real z);
    // Handles must be live/distinct. Caller supplies valid outward closed templates
    // and maintains a conforming manifold. No allocation-failure rollback guarantee.
    CellHandle add_hex(const std::array<VertexHandle, 8>& vertices,
                       const std::array<std::array<int, 4>, 6>& order = CellOrderTemplate::hex_faces);
    CellHandle add_tet(const std::array<VertexHandle, 4>& vertices,
                       const std::array<std::array<int, 3>, 4>& order = CellOrderTemplate::tet_faces);
    CellHandle add_pyramid(const std::array<VertexHandle, 5>& vertices,
                           const std::array<std::array<int, 4>, 5>& order = CellOrderTemplate::pyramid_faces);
    // Extension following the same interface, preserving existing prism support.
    CellHandle add_prism(const std::array<VertexHandle, 6>& vertices,
                         const std::array<std::array<int, 4>, 5>& order = CellOrderTemplate::prism_faces);
    bool delete_isolated_vertex(VertexHandle handle);
    bool delete_cell(CellHandle handle);
    // Collect handle snapshots before editing. Dense storage may move elements.
    HandleRange<VertexStorage, VertexHandle> vertices() const noexcept;
    HandleRange<DedgeStorage, DedgeHandle> dedges() const noexcept;
    HandleRange<HalffaceStorage, HalffaceHandle> halffaces() const noexcept;
    HandleRange<CellStorage, CellHandle> cells() const noexcept;
    const Vertex* vertex(VertexHandle handle) const noexcept;
    const Dedge* dedge(DedgeHandle handle) const noexcept;
    const Halfface* halfface(HalffaceHandle handle) const noexcept;
    const Cell* cell(CellHandle handle) const noexcept;
    Mesh copy() const;
    bool set_vertex_position(VertexHandle handle, Real x, Real y, Real z);
    std::vector<VertexHandle> vertex_vertices(VertexHandle handle) const;
    std::vector<CellHandle> edge_cells(DedgeHandle handle) const;
    // Positive templates: CCW looking from target to source. Boundary chain or closed
    // ring, no repeated first cell. Invalid handles, broken local links, duplicate
    // incidence, branching or disconnected stars return an empty vector.
    std::vector<CellHandle> edge_cells_ccw(DedgeHandle handle) const;
    std::vector<CellHandle> cell_cells(CellHandle handle) const;
private:
    template<std::size_t V, std::size_t F, std::size_t E>
    CellHandle insert_cell(CellType type, const std::array<VertexHandle, V>& vertices,
                           const std::array<std::array<int, E>, F>& order);
    DedgeHandle find_dedge(VertexHandle source, VertexHandle target) const;
    HalffaceHandle find_opposite(const std::vector<VertexHandle>& face) const;
    void swap(Mesh& other) noexcept;
    VertexStorage vertices_;
    DedgeStorage dedges_;
    HalffaceStorage halffaces_;
    CellStorage cells_;
};
}
