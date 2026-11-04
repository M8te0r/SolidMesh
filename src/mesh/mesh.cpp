#include <solidmesh/mesh/mesh.h>

#include <algorithm>
#include <cassert>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace SolidMesh
{
    namespace
    {
        struct CellSize
        {
            std::size_t vertices;
            std::size_t faces;
            std::size_t face_edges;
        };

        CellSize cell_size(CellType type)
        {
            switch (type)
            {
            case CellType::TET:     return {4, 4, 12};
            case CellType::HEX:     return {8, 6, 24};
            case CellType::PYRAMID: return {5, 5, 16};
            default: throw std::runtime_error("Unsupported cell type");
            }
        }

        IndexType next_id(std::size_t size)
        {
            if (size >= invalid_id)
                throw std::runtime_error("Mesh exceeds the element ID capacity");
            return static_cast<IndexType>(size);
        }

        using FaceKey = std::array<IndexType, 4>;

        struct FaceHash
        {
            std::size_t operator()(const FaceKey& key) const noexcept
            {
                std::size_t hash = 0;
                for (IndexType vi : key)
                    hash ^= std::size_t(vi) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
                return hash;
            }
        };

        struct EdgeUse
        {
            IndexType edge;
            // 本单元中使用该边 de0 的外向半面；跨过它就是 CCW 后继单元。
            IndexType halfface;
        };
    }

    Mesh Mesh::from_data(const std::vector<Vector3>& vertices,
                         const std::vector<std::vector<IndexType>>& cells,
                         const std::vector<CellType>& celltypes)
    {
        if (cells.size() != celltypes.size())
            throw std::runtime_error("Cell and cell type counts differ");
        if (vertices.size() > invalid_id || cells.size() > invalid_id)
            throw std::runtime_error("Mesh exceeds the element ID capacity");

        Mesh mesh;
        mesh.vertices_.reserve(vertices.size());
        for (const Vector3& point : vertices)
            mesh.vertices_.push_back(Vertex{point});

        mesh.cells_.resize(cells.size());
        std::size_t total_halffaces = 0;
        std::size_t total_face_edges = 0;
        for (std::size_t ci = 0; ci < cells.size(); ++ci)
        {
            const CellSize size = cell_size(celltypes[ci]);
            if (cells[ci].size() != size.vertices)
                throw std::runtime_error("Wrong vertex count in cell " + std::to_string(ci));
            if (total_halffaces > invalid_id - size.faces)
                throw std::runtime_error("Mesh exceeds the half-face ID capacity");
            total_halffaces += size.faces;
            total_face_edges += size.face_edges;

            Cell& cell = mesh.cells_[ci];
            cell.type = celltypes[ci];
            for (std::size_t i = 0; i < size.vertices; ++i)
            {
                if (cells[ci][i] >= vertices.size())
                    throw std::runtime_error("Invalid vertex ID in cell " + std::to_string(ci));
                cell.vertices[i] = cells[ci][i];
            }
        }

        mesh.halffaces_.reserve(total_halffaces);
        mesh.faces_.reserve(total_halffaces);
        mesh.edges_.reserve(total_face_edges / 2);
        mesh.dedges_.reserve(total_face_edges);
        std::vector<EdgeUse> edge_uses;
        edge_uses.reserve(total_face_edges / 2);

        // 哈希表只在构建元素时使用，不作为 Mesh 的持久存储。
        {
            std::unordered_map<std::uint64_t, IndexType> edge_map;
            std::unordered_map<FaceKey, IndexType, FaceHash> face_map;
            edge_map.reserve(total_face_edges / 2);
            face_map.reserve(total_halffaces);

            auto ensure_dedge = [&](IndexType source, IndexType target) {
                const IndexType lo = std::min(source, target);
                const IndexType hi = std::max(source, target);
                const std::uint64_t key = (std::uint64_t(lo) << 32) | hi;
                auto found = edge_map.find(key);
                if (found == edge_map.end())
                {
                    const IndexType ei = next_id(mesh.edges_.size());
                    const IndexType de0 = next_id(mesh.dedges_.size());
                    const IndexType de1 = next_id(mesh.dedges_.size() + 1);
                    mesh.edges_.push_back(Edge{de0, de1});
                    mesh.dedges_.push_back(DirectedEdge{{lo, hi}, de1, ei});
                    mesh.dedges_.push_back(DirectedEdge{{hi, lo}, de0, ei});
                    found = edge_map.emplace(key, ei).first;
                }
                const Edge& edge = mesh.edges_[found->second];
                return source == lo ? edge.de0 : edge.de1;
            };

            for (std::size_t ci = 0; ci < mesh.cells_.size(); ++ci)
            {
                Cell& cell = mesh.cells_[ci];
                std::size_t local_face = 0;
                auto add_halfface = [&](const auto& order) {
                    const IndexType hfi = next_id(mesh.halffaces_.size());
                    HalfFace hf;
                    hf.cell = static_cast<IndexType>(ci);
                    FaceKey ring{invalid_id, invalid_id, invalid_id, invalid_id};
                    std::size_t count = 0;
                    for (int local_vertex : order)
                        if (local_vertex >= 0)
                            ring[count++] = cell.vertices[local_vertex];

                    for (std::size_t j = 0; j < count; ++j)
                    {
                        const IndexType dei = ensure_dedge(ring[j], ring[(j + 1) % count]);
                        hf.dedges[j] = dei;
                        const IndexType ei = mesh.dedges_[dei].edge;
                        // 闭合单元模板中，每条物理边恰有一个面使用 de0。
                        if (dei == mesh.edges_[ei].de0)
                            edge_uses.push_back({ei, hfi});
                    }

                    FaceKey key = ring;
                    std::sort(key.begin(), key.begin() + count);
                    const auto found = face_map.find(key);
                    if (found == face_map.end())
                    {
                        hf.face = next_id(mesh.faces_.size());
                        mesh.faces_.push_back(Face{count == 3 ? FaceType::TRIANGLE : FaceType::QUAD,
                                                   {hfi, invalid_id}});
                        face_map.emplace(key, hf.face);
                    }
                    else
                    {
                        hf.face = found->second;
                        Face& face = mesh.faces_[hf.face];
                        if (face.halffaces[1] != invalid_id)
                            throw std::runtime_error("More than two cells share face " + std::to_string(hf.face));

                        HalfFace& other = mesh.halffaces_[face.halffaces[0]];
                        const IndexType reverse_first = mesh.dedges_[hf.dedges[0]].opposite;
                        const auto begin = other.dedges.begin();
                        const auto match = std::find(begin, begin + count, reverse_first);
                        if (match == begin + count)
                            throw std::runtime_error("Incompatible orientation at face " + std::to_string(hf.face));
                        const std::size_t start = static_cast<std::size_t>(match - begin);
                        for (std::size_t j = 0; j < count; ++j)
                            if (other.dedges[(start + count - j) % count] != mesh.dedges_[hf.dedges[j]].opposite)
                                throw std::runtime_error("Incompatible face loop at face " + std::to_string(hf.face));

                        hf.opposite = face.halffaces[0];
                        other.opposite = hfi;
                        face.halffaces[1] = hfi;
                    }
                    cell.halffaces[local_face++] = hfi;
                    mesh.halffaces_.push_back(hf);
                };

                switch (cell.type)
                {
                case CellType::TET:
                    for (const auto& face : CellOrderTemplate::tet_faces) add_halfface(face);
                    break;
                case CellType::HEX:
                    for (const auto& face : CellOrderTemplate::hex_faces) add_halfface(face);
                    break;
                case CellType::PYRAMID:
                    for (const auto& face : CellOrderTemplate::pyramid_faces) add_halfface(face);
                    break;
                default: break; // 已由 cell_size 检查。
                }
            }
        }

        // 顶点 -> 出向有向边 CSR：计数、前缀和、填充。
        mesh.vertex_dedge_offsets_.assign(vertices.size() + 1, 0);
        for (const DirectedEdge& de : mesh.dedges_)
            ++mesh.vertex_dedge_offsets_[std::size_t(de.vertices[0]) + 1];
        std::partial_sum(mesh.vertex_dedge_offsets_.begin(), mesh.vertex_dedge_offsets_.end(),
                         mesh.vertex_dedge_offsets_.begin());
        mesh.vertex_dedges_.resize(mesh.dedges_.size());
        {
            auto cursor = mesh.vertex_dedge_offsets_;
            for (std::size_t dei = 0; dei < mesh.dedges_.size(); ++dei)
                mesh.vertex_dedges_[cursor[mesh.dedges_[dei].vertices[0]]++] = static_cast<IndexType>(dei);
        }

        // 将临时的单元-边关联按物理边分组；组内仍保持递增 cell ID。
        std::vector<std::size_t> edge_offsets(mesh.edges_.size() + 1, 0);
        for (const EdgeUse& use : edge_uses)
            ++edge_offsets[std::size_t(use.edge) + 1];
        std::partial_sum(edge_offsets.begin(), edge_offsets.end(), edge_offsets.begin());
        std::vector<IndexType> edge_halffaces(edge_uses.size());
        {
            auto cursor = edge_offsets;
            for (const EdgeUse& use : edge_uses)
                edge_halffaces[cursor[use.edge]++] = use.halfface;
        }

        struct Link
        {
            IndexType cell;
            std::size_t next;
            bool has_predecessor = false;
            bool visited = false;
        };
        std::vector<Link> links;
        std::vector<IndexType> ordered;
        mesh.dedge_cell_offsets_.assign(mesh.dedges_.size() + 1, 0);
        mesh.dedge_cells_.reserve(edge_uses.size() * 2);
        for (std::size_t ei = 0; ei < mesh.edges_.size(); ++ei)
        {
            const std::size_t begin = edge_offsets[ei];
            const std::size_t count = edge_offsets[ei + 1] - begin;
            links.clear();
            ordered.clear();
            for (std::size_t i = 0; i < count; ++i)
                links.push_back({mesh.halffaces_[edge_halffaces[begin + i]].cell, count});

            auto fail = [ei]() {
                throw std::runtime_error("Cannot order cells around edge " + std::to_string(ei));
            };
            for (std::size_t i = 0; i < count; ++i)
            {
                const IndexType opposite = mesh.halffaces_[edge_halffaces[begin + i]].opposite;
                if (opposite == invalid_id) continue;
                const IndexType next_cell = mesh.halffaces_[opposite].cell;
                const auto next = std::lower_bound(links.begin(), links.end(), next_cell,
                    [](const Link& link, IndexType ci) { return link.cell < ci; });
                if (next == links.end() || next->cell != next_cell || next->has_predecessor) fail();
                next->has_predecessor = true;
                links[i].next = static_cast<std::size_t>(next - links.begin());
            }

            std::size_t start = 0;
            std::size_t roots = 0;
            for (std::size_t i = 0; i < count; ++i)
                if (!links[i].has_predecessor) { start = i; ++roots; }
            if (count == 0 || roots > 1) fail();
            const bool closed = roots == 0;

            // 闭环从最小 cell ID 开始；开链从唯一无前驱的单元开始。
            std::size_t current = start;
            for (std::size_t i = 0; i < count; ++i)
            {
                if (current == count || links[current].visited) fail();
                links[current].visited = true;
                ordered.push_back(links[current].cell);
                current = links[current].next;
            }
            if (current != (closed ? start : count)) fail();

            const Edge& edge = mesh.edges_[ei];
            mesh.dedge_cell_offsets_[edge.de0] = mesh.dedge_cells_.size();
            mesh.dedge_cells_.insert(mesh.dedge_cells_.end(), ordered.begin(), ordered.end());
            mesh.dedge_cell_offsets_[edge.de1] = mesh.dedge_cells_.size();
            if (closed)
            {
                mesh.dedge_cells_.push_back(ordered.front());
                mesh.dedge_cells_.insert(mesh.dedge_cells_.end(), ordered.rbegin(), ordered.rend() - 1);
            }
            else
                mesh.dedge_cells_.insert(mesh.dedge_cells_.end(), ordered.rbegin(), ordered.rend());
            mesh.dedge_cell_offsets_[std::size_t(edge.de1) + 1] = mesh.dedge_cells_.size();
        }
        return mesh;
    }

    std::size_t Mesh::num_vertices() const noexcept { return vertices_.size(); }
    std::size_t Mesh::num_cells() const noexcept { return cells_.size(); }
    std::size_t Mesh::num_faces() const noexcept { return faces_.size(); }
    std::size_t Mesh::num_halffaces() const noexcept { return halffaces_.size(); }
    std::size_t Mesh::num_edges() const noexcept { return edges_.size(); }
    std::size_t Mesh::num_dedges() const noexcept { return dedges_.size(); }

    const Vertex& Mesh::vertex(IndexType idx) const noexcept
    {
        assert(idx < vertices_.size());
        return vertices_[idx];
    }

    const DirectedEdge& Mesh::dedge(IndexType idx) const noexcept
    {
        assert(idx < dedges_.size());
        return dedges_[idx];
    }

    const Edge& Mesh::edge(IndexType idx) const noexcept
    {
        assert(idx < edges_.size());
        return edges_[idx];
    }

    const HalfFace& Mesh::halfface(IndexType idx) const noexcept
    {
        assert(idx < halffaces_.size());
        return halffaces_[idx];
    }

    const Face& Mesh::face(IndexType idx) const noexcept
    {
        assert(idx < faces_.size());
        return faces_[idx];
    }

    const Cell& Mesh::cell(IndexType idx) const noexcept
    {
        assert(idx < cells_.size());
        return cells_[idx];
    }

    std::vector<IndexType> Mesh::vertex_dedges(IndexType vi) const
    {
        if (vi >= vertices_.size()) return {};
        return {vertex_dedges_.begin() + vertex_dedge_offsets_[vi],
                vertex_dedges_.begin() + vertex_dedge_offsets_[std::size_t(vi) + 1]};
    }

    std::vector<IndexType> Mesh::vertex_vertices(IndexType vi) const
    {
        if (vi >= vertices_.size()) return {};
        const std::size_t begin = vertex_dedge_offsets_[vi];
        const std::size_t end = vertex_dedge_offsets_[std::size_t(vi) + 1];
        std::vector<IndexType> result;
        result.reserve(end - begin);
        for (std::size_t i = begin; i < end; ++i)
            result.push_back(dedges_[vertex_dedges_[i]].vertices[1]);
        return result;
    }

    std::vector<IndexType> Mesh::dedge_cells_ccw(IndexType dei) const
    {
        if (dei >= dedges_.size()) return {};
        return {dedge_cells_.begin() + dedge_cell_offsets_[dei],
                dedge_cells_.begin() + dedge_cell_offsets_[std::size_t(dei) + 1]};
    }

    std::vector<IndexType> Mesh::cell_cells(IndexType ci) const
    {
        if (ci >= cells_.size()) return {};
        std::vector<IndexType> result;
        result.reserve(6);
        for (IndexType hfi : cells_[ci].halffaces)
        {
            if (hfi == invalid_id) break;
            const IndexType opposite = halffaces_[hfi].opposite;
            if (opposite == invalid_id) continue;
            const IndexType neighbor = halffaces_[opposite].cell;
            if (neighbor != ci && std::find(result.begin(), result.end(), neighbor) == result.end())
                result.push_back(neighbor);
        }
        return result;
    }
} // namespace SolidMesh
