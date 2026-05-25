#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

#include <solidmesh/math/vector3.h>
#include <solidmesh/mesh/cell_type.h>

namespace SolidMesh
{

    
    static constexpr uint32_t invalid_id = std::numeric_limits<uint32_t>::max();

    struct Vertex
    {
        uint32_t id = invalid_id;
        Vector3 point = Vector3::undefined();
    };

    struct Cell
    {
        uint32_t id = invalid_id;
        CellType type = CellType::Invalid;
        int vertex_count = 0;
        int halfface_count = 0;
        std::array<uint32_t, 8> vertices = {
            invalid_id, invalid_id, invalid_id, invalid_id,
            invalid_id, invalid_id, invalid_id, invalid_id};

        std::array<uint32_t, 6> halffaces = {
            invalid_id,
            invalid_id,
            invalid_id,
            invalid_id,
            invalid_id,
            invalid_id};
    };

    struct Face
    {
        uint32_t id = invalid_id;
        uint32_t hf0 = invalid_id, hf1 = invalid_id; // halfface的id
    };

    struct HalfFace
    {
        uint32_t id = invalid_id;
        uint32_t cell = invalid_id;
        uint32_t face = invalid_id;
        int cell_local_face = -1;
        uint8_t vertex_count = 0;
        std::array<uint32_t, 4> v = {
            invalid_id,
            invalid_id,
            invalid_id,
            invalid_id};
        uint32_t opposite = invalid_id;
    };

    struct Edge
    {
        uint32_t id = invalid_id;
        uint32_t v0 = invalid_id, v1 = invalid_id;      // 端点id，规定v0<v1
        uint32_t de0 = invalid_id, de1 = invalid_id;    // DirectEdge的id
    };

    struct DirectedEdge
    {
        uint32_t id = invalid_id;
        uint32_t edge = invalid_id;                    // 所属边的id
        uint32_t start = invalid_id, end = invalid_id; // 起点、终点的vertex id
        uint32_t opposite = invalid_id;                // 反向DirectEdge的id
    };

    class Mesh
    {
    public:
        Mesh() = default;
        ~Mesh() = default;

    public:
        // 构造拓扑
        void build_topology(const std::vector<std::vector<double>> &points,
                            const std::vector<std::vector<uint32_t>> &cells,
                            const std::vector<CellType> &celltypes);

        // ---- 元素数量 -------------------------------------------------
        size_t num_vertices() const noexcept { return vertices_.size(); }
        size_t num_cells() const noexcept { return cells_.size(); }
        size_t num_faces() const noexcept { return faces_.size(); }
        size_t num_halffaces() const noexcept { return halffaces_.size(); }
        size_t num_edges() const noexcept { return edges_.size(); }
        size_t num_dedges() const noexcept { return dedges_.size(); }

        // ---- 查找 -------------------------------------------------
        const Vertex &vertex_at(uint32_t idx) const noexcept;
        const Cell &cell_at(uint32_t idx) const noexcept;
        const Face &face_at(uint32_t idx) const noexcept;
        const HalfFace &halfface_at(uint32_t idx) const noexcept;
        const Edge &edge_at(uint32_t idx) const noexcept;
        const DirectedEdge &dedge_at(uint32_t idx) const noexcept;

        // ---- topology queries ----------------------------------

        // 顶点的1-ring顶点
        std::vector<uint32_t> vertex_vertices(uint32_t vi) const;

        // 有向边的1-ring cell（右手定理，即从dedge的终点望向起点，，1-ring cell应该是ccw顺序的）
        std::vector<uint32_t> edge_cells_ccw(uint32_t dei) const;

        // 有向面上的顶点（严格 CCW 顺序）
        std::vector<uint32_t> halfface_vertices(uint32_t hfi) const;

        // Cell 邻接的周边 cells
        std::vector<uint32_t> cell_cells(uint32_t ci) const;

        // Cell上的所有定向面
        std::span<const uint32_t> cell_halffaces(uint32_t ci) const noexcept;

        // Cell 的vertices
        std::span<const uint32_t> cell_vertices(uint32_t ci) const noexcept;

    private:
        // ---- 元素容器 ----------------------------------
        std::vector<Vertex> vertices_;
        std::vector<Cell> cells_;
        std::vector<Face> faces_;
        std::vector<HalfFace> halffaces_;
        std::vector<Edge> edges_;
        std::vector<DirectedEdge> dedges_;

        // ---- Compressed Sparse Row ----------------------------------
        std::vector<uint32_t> vertex_vertex_offsets_;
        std::vector<uint32_t> vertex_vertices_;
        std::vector<uint32_t> dedge_cell_offsets_;
        std::vector<uint32_t> dedge_cells_;

    };
}
