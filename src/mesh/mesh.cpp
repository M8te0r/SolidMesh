#include <solidmesh/mesh/mesh.h>

#include <algorithm>
#include <cassert>
#include <span>
#include <stdexcept>
#include <unordered_map>

namespace SolidMesh
{
    namespace
    {
        struct FaceHash
        {
            std::size_t operator()(const std::array<IndexType, 4> &vertices) const noexcept
            {
                std::size_t hash = 0;
                for (IndexType vi : vertices)
                    hash ^= std::size_t(vi) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
                return hash;
            }
        };

        // 保留键中两个端点的顺序；无向边的键由调用方排序。
        struct EdgeHash
        {
            std::size_t operator()(const std::array<IndexType, 2> &vertices) const noexcept
            {
                std::size_t hash = 0;
                for (IndexType vi : vertices)
                    hash ^= std::size_t(vi) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
                return hash;
            }
        };
    }

    Mesh Mesh::from_data(const std::vector<Vector3> &vertices,
                         const std::vector<std::vector<IndexType>> &cells,
                         const std::vector<CellType> &celltypes)
    {
        // =====================================================
        // 0. 检查输入合法性
        // =====================================================
        if (cells.size() != celltypes.size())
            throw std::runtime_error("Cell and cell type counts differ");
        if (vertices.size() > invalid_id || cells.size() > invalid_id)
            throw std::runtime_error("Mesh exceeds the element ID capacity");

        // =====================================================
        // 1. 根据输入，初始化顶点和单元
        // =====================================================
        Mesh mesh;
        mesh.vertices_.reserve(vertices.size());
        mesh.cells_.resize(cells.size());

        for (const Vector3 &point : vertices)
            mesh.vertices_.push_back(Vertex{point});

        for (std::size_t ci = 0; ci < cells.size(); ++ci)
        {
            mesh.cells_[ci].type = celltypes[ci];
            std::copy(cells[ci].begin(), cells[ci].end(), mesh.cells_[ci].vertices.begin());
        }

        // =====================================================
        // 2. 构建边、有向边、面、半面
        // =====================================================
        mesh.halffaces_.reserve(cells.size() * 6);
        mesh.faces_.reserve(cells.size() * 6);
        mesh.dedges_.reserve(cells.size() * 12);
        mesh.edges_.reserve(cells.size() * 12);

        std::unordered_map<std::array<IndexType, 4>, IndexType, FaceHash> face_map;
        std::unordered_map<std::array<IndexType, 2>, IndexType, EdgeHash> dedge_map;
        std::unordered_map<std::array<IndexType, 2>, IndexType, EdgeHash> edge_map;

        for (std::size_t ci = 0; ci < mesh.cells_.size(); ++ci)
        {
            Cell &cell = mesh.cells_[ci];
            std::size_t face_count = CellTopology::cell_face_count(cell.type);

            for (std::size_t fi = 0; fi < face_count; ++fi)
            {
                // 取出当前面的模板，将局部编号转换为有序的全局顶点 ID。
                auto face_order = CellTopology::cell_face_order(cell.type, fi);
                std::array<IndexType, 4> face_vertices{invalid_id, invalid_id, invalid_id, invalid_id};
                std::size_t vertex_count = 0;
                for (int local_vertex : face_order)
                {
                    if (local_vertex == -1)
                        break; // PYRAMID 三角面的末尾填充项。
                    face_vertices[vertex_count++] = cell.vertices[local_vertex];
                }

                // 排序副本作为 facekey，原顶点环保留方向。HalfFace 本身不去重。
                auto face_key = face_vertices;
                std::sort(face_key.begin(), face_key.end());
                const auto [face_it, new_face] =
                    face_map.try_emplace(face_key, static_cast<IndexType>(mesh.faces_.size()));
                if (new_face)
                    mesh.faces_.push_back(Face{vertex_count == 3 ? FaceType::TRIANGLE : FaceType::QUAD});

                HalfFace halfface;
                halfface.cell = static_cast<IndexType>(ci);
                halfface.face = face_it->second;

                // 沿原顶点环构造有向边，最后一条边从末顶点回到首顶点。
                for (std::size_t i = 0; i < vertex_count; ++i)
                {
                    const IndexType source = face_vertices[i];
                    const IndexType target = face_vertices[(i + 1) % vertex_count];
                    const std::array<IndexType, 2> dedge_key{source, target};
                    const auto [dedge_it, new_dedge] = dedge_map.try_emplace(dedge_key, static_cast<IndexType>(mesh.dedges_.size()));
                    const IndexType dedge_id = dedge_it->second;

                    if (new_dedge)
                    {
                        DirectedEdge dedge;
                        dedge.vertices = dedge_key;

                        // 有向键区分 source→target；无向键合并两个方向。
                        const std::array<IndexType, 2> edge_key{std::min(source, target), std::max(source, target)};
                        const auto [edge_it, new_edge] = edge_map.try_emplace(edge_key, static_cast<IndexType>(mesh.edges_.size()));
                        if (new_edge)
                            mesh.edges_.emplace_back();
                        dedge.edge = edge_it->second;

                        // de0 为小 ID→大 ID，de1 为反向；第二个方向出现时配对。
                        Edge &edge = mesh.edges_[dedge.edge];
                        if (source < target)
                        {
                            edge.de0 = dedge_id;
                            dedge.opposite = edge.de1;
                        }
                        else
                        {
                            edge.de1 = dedge_id;
                            dedge.opposite = edge.de0;
                        }
                        if (dedge.opposite != invalid_id)
                            mesh.dedges_[dedge.opposite].opposite = dedge_id;
                        mesh.dedges_.push_back(dedge);
                    }
                    halfface.dedges[i] = dedge_id;
                }

                // 将半面登记到单元和面；共享同一 Face 的两个半面互为 opposite。
                const IndexType halfface_id = static_cast<IndexType>(mesh.halffaces_.size());
                Face &face = mesh.faces_[halfface.face];
                if (face.halffaces[0] == invalid_id)
                    face.halffaces[0] = halfface_id;
                else
                {
                    halfface.opposite = face.halffaces[0];
                    mesh.halffaces_[halfface.opposite].opposite = halfface_id;
                    face.halffaces[1] = halfface_id;
                }
                cell.halffaces[fi] = halfface_id;
                mesh.halffaces_.push_back(halfface);
            }
        }

        // =====================================================
        // 3. 构建vertex-dedge和dedge-cell的CSR索引
        // =====================================================
        // 3.1 vertex-dedge：按起点计数，累加为行边界，再填入有向边 ID。
        mesh.vertex_dedge_offsets_.assign(mesh.vertices_.size() + 1, 0);
        for (const DirectedEdge &dedge : mesh.dedges_)
            ++mesh.vertex_dedge_offsets_[std::size_t(dedge.vertices[0]) + 1];
        for (std::size_t i = 1; i < mesh.vertex_dedge_offsets_.size(); ++i)
            mesh.vertex_dedge_offsets_[i] += mesh.vertex_dedge_offsets_[i - 1];

        mesh.vertex_dedges_.resize(mesh.vertex_dedge_offsets_.back());
        {
            // 写入游标使用副本，保留 offsets 中的行边界。
            auto write_positions = mesh.vertex_dedge_offsets_;
            for (std::size_t di = 0; di < mesh.dedges_.size(); ++di)
            {
                const IndexType source = mesh.dedges_[di].vertices[0];
                mesh.vertex_dedges_[write_positions[source]++] = static_cast<IndexType>(di);
            }
        }

        // 3.2 dedge-cell：统计每行长度，同时确定沿 CCW 遍历的起始单元。
        // 合法单元的两个相邻半面使用相反方向的边，因此同一 dedge 每个单元只计一次。
        mesh.dedge_cell_offsets_.assign(mesh.dedges_.size() + 1, 0);
        std::vector<IndexType> start_cells(mesh.dedges_.size(), invalid_id);
        for (const HalfFace &halfface : mesh.halffaces_)
        {
            for (IndexType di : halfface.dedges)
            {
                if (di == invalid_id)
                    break;
                ++mesh.dedge_cell_offsets_[std::size_t(di) + 1];

                // 包含 di 的半面，是沿反向边 CCW 遍历时的入口。
                // 第 2 步按 cell ID 递增创建半面，首次遇到的单元即最小 ID；边界入口优先。
                const IndexType reverse_di = mesh.dedges_[di].opposite;
                if (start_cells[reverse_di] == invalid_id || halfface.opposite == invalid_id)
                    start_cells[reverse_di] = halfface.cell;
            }
        }
        for (std::size_t i = 1; i < mesh.dedge_cell_offsets_.size(); ++i)
            mesh.dedge_cell_offsets_[i] += mesh.dedge_cell_offsets_[i - 1];

        mesh.dedge_cells_.resize(mesh.dedge_cell_offsets_.back());

        // 3.3 沿半面连接直接填入有序单元。每条 Edge 只遍历 de0，de1 由反向序列得到。
        for (const Edge &edge : mesh.edges_)
        {
            const std::size_t begin = mesh.dedge_cell_offsets_[edge.de0];
            const std::size_t end = mesh.dedge_cell_offsets_[std::size_t(edge.de0) + 1];
            const IndexType start_cell = start_cells[edge.de0];

            // 跨过当前单元中包含 de0 的半面，即沿约定的 CCW 方向进入下一个单元。
            IndexType current_cell = start_cell;
            for (std::size_t i = begin; i < end; ++i)
            {
                mesh.dedge_cells_[i] = current_cell;
                for (IndexType hfi : mesh.cells_[current_cell].halffaces)
                {
                    if (hfi == invalid_id)
                        break;
                    const HalfFace &halfface = mesh.halffaces_[hfi];
                    if (std::find(halfface.dedges.begin(), halfface.dedges.end(), edge.de0) == halfface.dedges.end())
                        continue;
                    current_cell = halfface.opposite == invalid_id
                        ? invalid_id : mesh.halffaces_[halfface.opposite].cell;
                    break;
                }
            }

            const std::size_t reverse_begin = mesh.dedge_cell_offsets_[edge.de1];
            // 开链在边界终止，闭环走完后回到起点。
            if (current_cell == invalid_id)
            {
                std::reverse_copy(mesh.dedge_cells_.begin() + begin, mesh.dedge_cells_.begin() + end,
                                  mesh.dedge_cells_.begin() + reverse_begin);
            }
            else
            {
                // 反向闭环仍保留最小 cell ID 为首项，只反转其余部分。
                mesh.dedge_cells_[reverse_begin] = start_cell;
                std::reverse_copy(mesh.dedge_cells_.begin() + begin + 1, mesh.dedge_cells_.begin() + end,
                                  mesh.dedge_cells_.begin() + reverse_begin + 1);
            }
        }

        return mesh;
    }

    std::size_t Mesh::num_vertices() const noexcept { return vertices_.size(); }
    std::size_t Mesh::num_cells() const noexcept { return cells_.size(); }
    std::size_t Mesh::num_faces() const noexcept { return faces_.size(); }
    std::size_t Mesh::num_halffaces() const noexcept { return halffaces_.size(); }
    std::size_t Mesh::num_edges() const noexcept { return edges_.size(); }
    std::size_t Mesh::num_dedges() const noexcept { return dedges_.size(); }

    const Vertex &Mesh::vertex(IndexType idx) const noexcept
    {
        assert(idx < vertices_.size());
        return vertices_[idx];
    }

    const DirectedEdge &Mesh::dedge(IndexType idx) const noexcept
    {
        assert(idx < dedges_.size());
        return dedges_[idx];
    }

    const Edge &Mesh::edge(IndexType idx) const noexcept
    {
        assert(idx < edges_.size());
        return edges_[idx];
    }

    const HalfFace &Mesh::halfface(IndexType idx) const noexcept
    {
        assert(idx < halffaces_.size());
        return halffaces_[idx];
    }

    const Face &Mesh::face(IndexType idx) const noexcept
    {
        assert(idx < faces_.size());
        return faces_[idx];
    }

    const Cell &Mesh::cell(IndexType idx) const noexcept
    {
        assert(idx < cells_.size());
        return cells_[idx];
    }

    std::vector<IndexType> Mesh::vertex_dedges(IndexType vi) const
    {
        if (vi >= vertices_.size())
            return {};
        return {vertex_dedges_.begin() + vertex_dedge_offsets_[vi],
                vertex_dedges_.begin() + vertex_dedge_offsets_[std::size_t(vi) + 1]};
    }

    std::vector<IndexType> Mesh::vertex_vertices(IndexType vi) const
    {
        if (vi >= vertices_.size())
            return {};
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
        if (dei >= dedges_.size())
            return {};
        return {dedge_cells_.begin() + dedge_cell_offsets_[dei],
                dedge_cells_.begin() + dedge_cell_offsets_[std::size_t(dei) + 1]};
    }

    std::vector<IndexType> Mesh::cell_cells(IndexType ci) const
    {
        if (ci >= cells_.size())
            return {};
        std::vector<IndexType> result;
        result.reserve(6);
        for (IndexType hfi : cells_[ci].halffaces)
        {
            if (hfi == invalid_id)
                break;
            const IndexType opposite = halffaces_[hfi].opposite;
            if (opposite == invalid_id)
                continue;
            const IndexType neighbor = halffaces_[opposite].cell;
            if (neighbor != ci && std::find(result.begin(), result.end(), neighbor) == result.end())
                result.push_back(neighbor);
        }
        return result;
    }
} // namespace SolidMesh
