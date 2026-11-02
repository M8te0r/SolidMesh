#include <solidmesh/mesh/mesh.h>

#include <algorithm>
#include <cassert>

namespace SolidMesh
{
    void Mesh::build_topology(const std::vector<std::vector<double>> &points,
                              const std::vector<std::vector<uint32_t>> &cells,
                              const std::vector<CellType> &celltypes)
    {
        vertices_.clear();
        cells_.clear();
        faces_.clear();
        halffaces_.clear();
        edges_.clear();
        dedges_.clear();
        vertex_vertex_offsets_.clear();
        vertex_vertices_.clear();
        dedge_cell_offsets_.clear();
        dedge_cells_.clear();

        assert(cells.size() == celltypes.size());

        // 顶点数据
        vertices_.resize(points.size());
        for (uint32_t vi = 0; vi < points.size(); ++vi)
        {
            vertices_[vi].id = vi;
            vertices_[vi].point = {points[vi][0], points[vi][1], points[vi][2]};
        }

        // 单元数据
        cells_.resize(cells.size());
        for (uint32_t ci = 0; ci < cells.size(); ++ci)
        {
            Cell &cell = cells_[ci];
            cell.id = ci;
            cell.type = celltypes[ci];
            cell.vertex_count = cell_vertex_count(cell.type);
            cell.halfface_count = cell_face_count(cell.type);

            for (size_t i = 0; i < cells[ci].size(); ++i)
            {
                cell.vertices[i] = cells[ci][i];
            }
        }

        // ---------------------------------------------------------------------------------------
        // 去重，构建face key、edge key用于排序
        // ---------------------------------------------------------------------------------------
        struct FaceKey
        {
            std::array<uint32_t, 4> v = {invalid_id, invalid_id, invalid_id, invalid_id};
            int count = 0;

            bool operator==(const FaceKey &other) const noexcept
            {
                return count == other.count && v == other.v;
            }
        };

        struct FaceKeyHash
        {
            size_t operator()(const FaceKey &key) const noexcept
            {
                size_t h = key.count;
                for (int i = 0; i < key.count; ++i)
                {
                    h ^= static_cast<size_t>(key.v[i]) + 0x9e3779b9u + (h << 6) + (h >> 2);
                }
                return h;
            }
        };

        auto sort_key = [](FaceKey &key) noexcept { // 顶点相同（即使顺序不同）的面是一个面
            for (int i = 1; i < key.count; ++i)
            {
                const uint32_t value = key.v[i];
                int j = i;
                while (j > 0 && key.v[j - 1] > value)
                {
                    key.v[j] = key.v[j - 1];
                    --j;
                }
                key.v[j] = value;
            }
        };

        auto edge_key = [](uint32_t a, uint32_t b) noexcept -> uint64_t {
            if (a > b)
            {
                const uint32_t tmp = a;
                a = b;
                b = tmp;
            }
            return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b); // 拼接两个u32为一个u64去重
        };

        struct EdgeCellRef
        {
            uint32_t edge = invalid_id;
            // The cell's outward half-face containing edge.v0 -> edge.v1.
            uint32_t halfface = invalid_id;
        };

        std::unordered_map<FaceKey, uint32_t, FaceKeyHash> face_map;
        std::unordered_map<uint64_t, uint32_t> edge_map;

        // ---------------------------------------------------------------------------------------
        // 统计halfface数和face edge数用于预分配内存
        // ---------------------------------------------------------------------------------------

        size_t total_halffaces = 0;
        size_t total_face_edges = 0;
        for (const Cell &cell : cells_)
        {
            total_halffaces += cell.halfface_count;
            const CellTopoPadded &topo = cell_topo(cell.type);
            for (int local_face = 0; local_face < cell.halfface_count; ++local_face)
            {
                total_face_edges += topo.face_vertex_count[local_face];
            }
        }

        halffaces_.reserve(total_halffaces);
        faces_.reserve(total_halffaces);
        edges_.reserve(total_face_edges / 2);
        dedges_.reserve(total_face_edges);
        face_map.reserve(total_halffaces * 2);
        edge_map.reserve(total_face_edges);
        std::vector<EdgeCellRef> edge_cell_refs;
        edge_cell_refs.reserve(total_face_edges / 2);
        std::vector<uint8_t> nonmanifold_edges;
        nonmanifold_edges.reserve(total_face_edges / 2);

        auto ensure_dedge = [&](uint32_t start, uint32_t end) -> uint32_t { 
            const uint64_t key = edge_key(start, end);
            const auto found = edge_map.find(key);
            if (found != edge_map.end())    // 边存在，则
            {
                const Edge &edge = edges_[found->second];
                return (edge.v0 == start) ? edge.de0 : edge.de1;
            }

            const uint32_t v0 = (start < end) ? start : end;
            const uint32_t v1 = (start < end) ? end : start;
            const uint32_t edge_id = static_cast<uint32_t>(edges_.size());
            const uint32_t de0_id = static_cast<uint32_t>(dedges_.size());
            const uint32_t de1_id = de0_id + 1;

            Edge edge;
            edge.id = edge_id;
            edge.v0 = v0;
            edge.v1 = v1;
            edge.de0 = de0_id;
            edge.de1 = de1_id;
            edges_.push_back(edge);
            nonmanifold_edges.push_back(false);

            DirectedEdge de0;
            de0.id = de0_id;
            de0.edge = edge_id;
            de0.start = v0;
            de0.end = v1;
            de0.opposite = de1_id;
            dedges_.push_back(de0);

            DirectedEdge de1;
            de1.id = de1_id;
            de1.edge = edge_id;
            de1.start = v1;
            de1.end = v0;
            de1.opposite = de0_id;
            dedges_.push_back(de1);

            edge_map.emplace(key, edge_id);
            return (start == v0) ? de0_id : de1_id;
        };

        for (Cell &cell : cells_)
        {
            cell.halffaces.fill(invalid_id);

            const CellTopoPadded &topo = cell_topo(cell.type);
            std::array<uint32_t, cell_traits_max_cell_face_count * cell_traits_max_face_vertex_count> cell_edges{};
            std::array<uint32_t, cell_traits_max_cell_face_count * cell_traits_max_face_vertex_count> cell_edge_halffaces{};
            int cell_edge_count = 0;

            auto register_cell_edge = [&](uint32_t de_id, uint32_t hf_id) {
                const uint32_t edge_id = dedges_[de_id].edge;
                int i = 0;
                for (; i < cell_edge_count; ++i)
                {
                    if (cell_edges[i] == edge_id)
                    {
                        break;
                    }
                }
                if (i == cell_edge_count)
                {
                    cell_edges[i] = edge_id;
                    cell_edge_halffaces[i] = invalid_id;
                    ++cell_edge_count;
                }
                if (de_id == edges_[edge_id].de0)
                {
                    cell_edge_halffaces[i] = hf_id;
                }
            };

            for (int local_face = 0; local_face < cell.halfface_count; ++local_face)
            {
                const int vertex_count = topo.face_vertex_count[local_face];
                const uint32_t hf_id = static_cast<uint32_t>(halffaces_.size());

                HalfFace halfface;
                halfface.id = hf_id;
                halfface.cell = cell.id;
                halfface.cell_local_face = local_face;
                halfface.vertex_count = vertex_count;

                FaceKey key;
                key.count = vertex_count;
                for (int i = 0; i < vertex_count; ++i)
                {
                    const uint32_t vi = cell.vertices[topo.local_faces[local_face][i]];
                    halfface.v[i] = vi;
                    key.v[i] = vi;
                }
                sort_key(key);

                bool nonmanifold_face = false;
                const auto face_found = face_map.find(key);
                if (face_found == face_map.end())   // 创建新face，并绑定第一个hafface
                {
                    const uint32_t face_id = static_cast<uint32_t>(faces_.size());
                    Face face;
                    face.id = face_id;
                    face.hf0 = hf_id;
                    faces_.push_back(face);
                    face_map.emplace(key, face_id);
                    halfface.face = face_id;
                }
                else    // 更新当前halfface的opposite
                {
                    const uint32_t face_id = face_found->second;
                    Face &face = faces_[face_id];
                    halfface.face = face_id;
                    if (face.hf1 == invalid_id)
                    {
                        face.hf1 = hf_id;
                        halfface.opposite = face.hf0;
                        halffaces_[face.hf0].opposite = hf_id;
                    }
                    else
                    {
                        nonmanifold_face = true;
                    }
                }

                for (int i = 0; i < vertex_count; ++i)
                {
                    const uint32_t de_id = ensure_dedge(halfface.v[i], halfface.v[(i + 1) % vertex_count]);    // 注册halfface上的每条directed edge
                    register_cell_edge(de_id, hf_id);
                    if (nonmanifold_face)
                    {
                        // An unpaired third face must not masquerade as a
                        // boundary endpoint in an otherwise complete chain.
                        nonmanifold_edges[dedges_[de_id].edge] = true;
                    }
                }

                cell.halffaces[local_face] = hf_id;
                halffaces_.push_back(halfface);
            }

            for (int i = 0; i < cell_edge_count; ++i)
            {
                edge_cell_refs.push_back({cell_edges[i], cell_edge_halffaces[i]});
            }
        }

        // ---- Compressed Sparse Row ----------------------------------
        vertex_vertex_offsets_.assign(vertices_.size() + 1, 0);
        for (const Edge &edge : edges_) // 1. 统计每个顶点的度（连接的边数），存入偏移数组
        {
            ++vertex_vertex_offsets_[edge.v0 + 1];
            ++vertex_vertex_offsets_[edge.v1 + 1];
        }
        for (size_t i = 1; i < vertex_vertex_offsets_.size(); ++i)  // 2. 前缀和（Prefix Sum）：把“数量”转化为大数组中的“起始内存地址”
        {
            vertex_vertex_offsets_[i] += vertex_vertex_offsets_[i - 1];
        }

        vertex_vertices_.assign(edges_.size() * 2, invalid_id);
        std::vector<uint32_t> vertex_cursor = vertex_vertex_offsets_;
        for (const Edge &edge : edges_) //3. 填入真正的邻居顶点 ID
        {
            vertex_vertices_[vertex_cursor[edge.v0]++] = edge.v1;
            vertex_vertices_[vertex_cursor[edge.v1]++] = edge.v0;
        }

        std::vector<uint32_t> edge_cell_offsets(edges_.size() + 1, 0);
        for (const EdgeCellRef &ref : edge_cell_refs)
        {
            ++edge_cell_offsets[ref.edge + 1];
        }
        for (size_t i = 1; i < edge_cell_offsets.size(); ++i)
        {
            edge_cell_offsets[i] += edge_cell_offsets[i - 1];
        }

        std::vector<uint32_t> edge_halffaces(edge_cell_refs.size(), invalid_id);
        std::vector<uint32_t> edge_cursor = edge_cell_offsets;
        for (const EdgeCellRef &ref : edge_cell_refs)
        {
            edge_halffaces[edge_cursor[ref.edge]++] = ref.halfface;
        }

        // Crossing the outward face along v0 -> v1 gives the CCW successor,
        // viewed from v1 towards v0. This also determines the boundary chain's
        // start, which cannot be recovered from sorting centroid angles.
        struct CellLink
        {
            uint32_t cell = invalid_id;
            uint32_t successor = invalid_id;
            uint32_t next = invalid_id;
            bool has_predecessor = false;
            bool visited = false;
        };

        std::vector<CellLink> links;
        std::vector<uint32_t> ordered_cells;
        auto order_edge = [&](const Edge &edge, bool &closed) -> bool {
            links.clear();
            ordered_cells.clear();
            closed = false;
            if (edge.v0 == edge.v1 || nonmanifold_edges[edge.id])
            {
                return false;
            }

            const uint32_t begin = edge_cell_offsets[edge.id];
            const uint32_t end = edge_cell_offsets[edge.id + 1];
            links.reserve(end - begin);
            ordered_cells.reserve(end - begin);
            for (uint32_t i = begin; i < end; ++i)
            {
                const uint32_t hfi = edge_halffaces[i];
                if (hfi >= halffaces_.size())
                {
                    return false;
                }
                const HalfFace &hf = halffaces_[hfi];
                const Face &face = faces_[hf.face];
                if (face.hf0 != hfi && face.hf1 != hfi)
                {
                    return false; // More than two half-faces share this face.
                }
                uint32_t successor = invalid_id;
                if (hf.opposite != invalid_id)
                {
                    const HalfFace &opposite = halffaces_[hf.opposite];
                    if (opposite.opposite != hfi || opposite.face != hf.face || opposite.cell == hf.cell)
                    {
                        return false;
                    }
                    bool reversed_edge = false;
                    for (uint32_t j = 0; j < opposite.vertex_count; ++j)
                    {
                        reversed_edge |= opposite.v[j] == edge.v1 &&
                                         opposite.v[(j + 1) % opposite.vertex_count] == edge.v0;
                    }
                    if (!reversed_edge)
                    {
                        return false; // Incompatible orientation on a shared face.
                    }
                    successor = opposite.cell;
                }
                // References are appended in cell order before grouping by edge.
                if (!links.empty() && links.back().cell >= hf.cell)
                {
                    return false;
                }
                links.push_back({hf.cell, successor});
            }
            if (links.empty())
            {
                return false;
            }

            size_t ends = 0;
            for (CellLink &link : links)
            {
                if (link.successor == invalid_id)
                {
                    ++ends;
                    continue;
                }
                auto next = std::lower_bound(links.begin(), links.end(), link.successor,
                                            [](const CellLink &a, uint32_t cell) { return a.cell < cell; });
                if (next == links.end() || next->cell != link.successor || next->has_predecessor)
                {
                    return false;
                }
                next->has_predecessor = true;
                link.next = static_cast<uint32_t>(next - links.begin());
            }

            size_t starts = 0;
            uint32_t start = 0;
            for (uint32_t i = 0; i < links.size(); ++i)
            {
                if (!links[i].has_predecessor)
                {
                    start = i;
                    ++starts;
                }
            }
            closed = starts == 0 && ends == 0;
            if (!closed && (starts != 1 || ends != 1))
            {
                return false;
            }
            // A ring starts at the smallest cell id; a chain at its unique root.
            uint32_t current = start;
            for (size_t i = 0; i < links.size(); ++i)
            {
                if (current == invalid_id || links[current].visited)
                {
                    return false;
                }
                CellLink &link = links[current];
                link.visited = true;
                ordered_cells.push_back(link.cell);
                current = link.next;
            }
            return closed ? current == start : current == invalid_id;
        };

        // Keep the existing CSR query layout. Each physical edge creates its two
        // directed ids consecutively; generate the reverse order without walking
        // the star a second time. Invalid local stars get two empty CSR ranges.
        dedge_cell_offsets_.assign(dedges_.size() + 1, 0);
        dedge_cells_.reserve(edge_cell_refs.size() * 2);
        for (const Edge &edge : edges_)
        {
            assert(edge.de1 == edge.de0 + 1);
            dedge_cell_offsets_[edge.de0] = static_cast<uint32_t>(dedge_cells_.size());
            bool closed = false;
            const bool valid = order_edge(edge, closed);
            if (valid)
            {
                dedge_cells_.insert(dedge_cells_.end(), ordered_cells.begin(), ordered_cells.end());
            }
            dedge_cell_offsets_[edge.de1] = static_cast<uint32_t>(dedge_cells_.size());

            if (valid && closed)
            {
                dedge_cells_.push_back(ordered_cells.front());
                for (size_t i = ordered_cells.size(); i > 1; --i)
                {
                    dedge_cells_.push_back(ordered_cells[i - 1]);
                }
            }
            else if (valid)
            {
                dedge_cells_.insert(dedge_cells_.end(), ordered_cells.rbegin(), ordered_cells.rend());
            }
            dedge_cell_offsets_[edge.de1 + 1] = static_cast<uint32_t>(dedge_cells_.size());
        }
    }

    const Vertex &Mesh::vertex_at(uint32_t idx) const noexcept
    {
        return vertices_[idx];
    }

    const Cell &Mesh::cell_at(uint32_t idx) const noexcept
    {
        return cells_[idx];
    }

    const Face &Mesh::face_at(uint32_t idx) const noexcept
    {
        return faces_[idx];
    }

    const HalfFace &Mesh::halfface_at(uint32_t idx) const noexcept
    {
        return halffaces_[idx];
    }

    const Edge &Mesh::edge_at(uint32_t idx) const noexcept
    {
        return edges_[idx];
    }

    const DirectedEdge &Mesh::dedge_at(uint32_t idx) const noexcept
    {
        return dedges_[idx];
    }

    std::vector<uint32_t> Mesh::vertex_vertices(uint32_t vi) const
    {
        std::vector<uint32_t> result;
        const uint32_t begin = vertex_vertex_offsets_[vi];
        const uint32_t end = vertex_vertex_offsets_[vi + 1];
        result.reserve(end - begin);
        result.insert(result.end(), vertex_vertices_.begin() + begin, vertex_vertices_.begin() + end);
        return result;
    }

    std::vector<uint32_t> Mesh::edge_cells_ccw(uint32_t dei) const
    {
        if (dei >= dedges_.size())
        {
            return {};
        }
        std::vector<uint32_t> result;
        const uint32_t begin = dedge_cell_offsets_[dei];
        const uint32_t end = dedge_cell_offsets_[dei + 1];
        result.reserve(end - begin);
        result.insert(result.end(), dedge_cells_.begin() + begin, dedge_cells_.begin() + end);
        return result;
    }

    std::vector<uint32_t> Mesh::halfface_vertices(uint32_t hfi) const
    {
        const HalfFace &halfface = halffaces_[hfi];
        std::vector<uint32_t> result;
        result.reserve(halfface.vertex_count);
        for (int i = 0; i < halfface.vertex_count; ++i)
        {
            result.push_back(halfface.v[i]);
        }
        return result;
    }

    std::vector<uint32_t> Mesh::cell_cells(uint32_t ci) const
    {
        const Cell &cell = cells_[ci];
        std::vector<uint32_t> result;
        result.reserve(cell.halfface_count);
        for (int i = 0; i < cell.halfface_count; ++i)
        {
            const HalfFace &halfface = halffaces_[cell.halffaces[i]];
            if (halfface.opposite != invalid_id)
            {
                result.push_back(halffaces_[halfface.opposite].cell);
            }
        }
        return result;
    }

    std::span<const uint32_t> Mesh::cell_halffaces(uint32_t ci) const noexcept
    {
        const Cell &cell = cells_[ci];
        return std::span<const uint32_t>(cell.halffaces.data(), static_cast<size_t>(cell.halfface_count));
    }

    std::span<const uint32_t> Mesh::cell_vertices(uint32_t ci) const noexcept
    {
        const Cell &cell = cells_[ci];
        return std::span<const uint32_t>(cell.vertices.data(), static_cast<size_t>(cell.vertex_count));
    }
}
