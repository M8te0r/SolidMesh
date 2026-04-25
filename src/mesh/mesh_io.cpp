#include "solidmesh/mesh/mesh_io.h"
#include "solidmesh/mesh/io_helper.h"

#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace SolidMesh {
namespace {

struct VtkCellOutput {
    std::vector<std::vector<size_t>> cells;
    std::vector<int> cell_types;
    size_t total_size = 0;
};

bool read_size(FastAsciiReader& reader, size_t& value) {
    std::uint64_t raw = 0;
    if (!reader.read_uint64(raw)) return false;
    if (raw > static_cast<std::uint64_t>(std::numeric_limits<size_t>::max()))
        return false;
    value = static_cast<size_t>(raw);
    return true;
}

bool vtk_cell_type_to_cell_type(int vtk_type, CellType& cell_type, int& expected_nverts) {
    switch (vtk_type) {
        case 10: cell_type = CellType::Tet;     expected_nverts = 4; return true;
        case 12: cell_type = CellType::Hex;     expected_nverts = 8; return true;
        case 13: cell_type = CellType::Prism;   expected_nverts = 6; return true;
        case 14: cell_type = CellType::Pyramid; expected_nverts = 5; return true;
        default: return false;
    }
}

int cell_type_to_vtk_type(CellType cell_type) {
    switch (cell_type) {
        case CellType::Tet:     return 10;
        case CellType::Hex:     return 12;
        case CellType::Prism:   return 13;
        case CellType::Pyramid: return 14;
        default: return -1;
    }
}

bool build_mesh_from_vtk_data(const std::vector<Vector3>& points,
                              const std::vector<size_t>& cell_offsets,
                              const std::vector<int>& cell_conn,
                              const std::vector<int>& cell_types,
                              PolyhedralMesh& mesh) {
    if (cell_offsets.empty()) return false;
    const size_t num_cells = cell_offsets.size() - 1;
    if (cell_types.size() != num_cells) return false;

    PolyhedralMesh loaded;
    std::vector<VertexHandle> vtk_vertices;
    vtk_vertices.reserve(points.size());
    for (const Vector3& p : points) {
        vtk_vertices.push_back(loaded.add_vertex(p));
    }

    std::vector<VertexHandle> cell_verts;
    cell_verts.reserve(8);

    for (size_t ci = 0; ci < num_cells; ++ci) {
        CellType ctype = CellType::Tet;
        int expected_nverts = 0;

        if (!vtk_cell_type_to_cell_type(cell_types[ci], ctype, expected_nverts)) {
            continue; // Unsupported VTK type -> silently skip.
        }

        const size_t begin = cell_offsets[ci];
        const size_t end = cell_offsets[ci + 1];
        if (begin > end || end > cell_conn.size()) return false;

        const size_t nv = end - begin;
        if (nv != static_cast<size_t>(expected_nverts)) return false;

        cell_verts.clear();
        for (size_t k = begin; k < end; ++k) {
            const int vtk_vid = cell_conn[k];
            if (vtk_vid < 0 || static_cast<size_t>(vtk_vid) >= vtk_vertices.size())
                return false;
            cell_verts.push_back(vtk_vertices[static_cast<size_t>(vtk_vid)]);
        }

        CellHandle cell = loaded.add_cell(ctype, cell_verts);
        if (!loaded.is_handle_valid(cell)) return false;
    }

    mesh = std::move(loaded);
    return true;
}

bool collect_vtk_cell_output(const PolyhedralMesh& mesh,
                             const std::unordered_map<uint32_t, size_t>& slot_to_idx,
                             VtkCellOutput& output) {
    output.cells.clear();
    output.cell_types.clear();
    output.total_size = 0;
    output.cells.reserve(mesh.num_cells());
    output.cell_types.reserve(mesh.num_cells());

    for (auto ch : mesh.cells()) {
        const int vtk_type = cell_type_to_vtk_type(ch.type());
        if (vtk_type < 0) continue; // skip unsupported

        std::vector<VertexHandle> verts = ch.vertices();
        std::vector<size_t> cverts;
        cverts.reserve(verts.size());
        for (const auto& vh : verts) {
            auto it = slot_to_idx.find(vh.id().slot);
            if (it == slot_to_idx.end()) return false;
            cverts.push_back(it->second);
        }

        output.total_size += cverts.size() + 1;
        output.cells.push_back(std::move(cverts));
        output.cell_types.push_back(vtk_type);
    }

    return true;
}

} // namespace

bool MeshIO::read_vtk_fast(const std::string& path, PolyhedralMesh& mesh) {
    try {
        FastAsciiReader in(path);

        // Legacy VTK header (4 lines):
        //   # vtk DataFile Version x.x
        //   <title>
        //   ASCII
        //   DATASET UNSTRUCTURED_GRID
        std::string line;
        if (!in.read_line(line)) return false;
        if (!in.read_line(line)) return false;
        if (!in.read_line(line)) return false;
        if (line.find("ASCII") == std::string::npos && line.find("ascii") == std::string::npos) return false;
        if (!in.read_line(line)) return false;
        if (line.find("UNSTRUCTURED_GRID") == std::string::npos) return false;

        std::vector<Vector3> points;
        std::vector<size_t> cell_offsets;
        std::vector<int> cell_conn;
        std::vector<int> cell_types;

        bool have_points = false;
        bool have_cells = false;
        bool have_cell_types = false;

        std::string token;
        while (in.read_token(token)) {
            if (token == "POINTS") {
                size_t num_points = 0;
                std::string scalar_type;
                if (!read_size(in, num_points)) return false;
                if (!in.read_token(scalar_type)) return false;

                points.clear();
                points.reserve(num_points);

                for (size_t i = 0; i < num_points; ++i) {
                    double x = 0.0, y = 0.0, z = 0.0;
                    if (!in.read_double(x)) return false;
                    if (!in.read_double(y)) return false;
                    if (!in.read_double(z)) return false;
                    points.push_back(Vector3{x, y, z});
                }
                have_points = true;
                continue;
            }

            if (token == "CELLS") {
                size_t num_cells = 0, total_size = 0;
                if (!read_size(in, num_cells)) return false;
                if (!read_size(in, total_size)) return false;
                if (total_size < num_cells) return false;

                cell_offsets.clear();
                cell_offsets.resize(num_cells + 1);
                cell_conn.clear();
                cell_conn.reserve(total_size - num_cells);

                size_t consumed = 0;
                for (size_t ci = 0; ci < num_cells; ++ci) {
                    int nverts = 0;
                    if (!in.read_int(nverts)) return false;
                    if (nverts < 0) return false;

                    cell_offsets[ci] = cell_conn.size();
                    consumed += static_cast<size_t>(nverts) + 1;

                    for (int j = 0; j < nverts; ++j) {
                        int v = -1;
                        if (!in.read_int(v)) return false;
                        if (v < 0) return false;
                        cell_conn.push_back(v);
                    }
                }
                cell_offsets[num_cells] = cell_conn.size();
                if (consumed != total_size) return false;

                have_cells = true;
                continue;
            }

            if (token == "CELL_TYPES") {
                size_t num_types = 0;
                if (!read_size(in, num_types)) return false;

                cell_types.clear();
                cell_types.resize(num_types);
                for (size_t i = 0; i < num_types; ++i) {
                    if (!in.read_int(cell_types[i])) return false;
                }

                have_cell_types = true;
                if (have_points && have_cells) break;
                continue;
            }
        }

        if (!have_points || !have_cells || !have_cell_types) return false;
        return build_mesh_from_vtk_data(points, cell_offsets, cell_conn, cell_types, mesh);
    } catch (...) {
        return false;
    }
}

bool MeshIO::write_vtk_fast(const std::string& path, const PolyhedralMesh& mesh) {
    try {
        FastAsciiWriter out(path);

        if (!out.write_string("# vtk DataFile Version 3.0\n")) return false;
        if (!out.write_string("SolidMesh export\n")) return false;
        if (!out.write_string("ASCII\n")) return false;
        if (!out.write_string("DATASET UNSTRUCTURED_GRID\n")) return false;

        const std::size_t num_points = mesh.num_vertices();
        if (!out.write_string("POINTS ")) return false;
        if (!out.write_uint64(static_cast<std::uint64_t>(num_points))) return false;
        if (!out.write_string(" double\n")) return false;

        std::unordered_map<uint32_t, size_t> slot_to_idx;
        slot_to_idx.reserve(num_points);

        size_t idx = 0;
        for (auto vh : mesh.vertices()) {
            const Vector3 p = vh.position();
            if (!out.write_double(p.x)) return false;
            if (!out.write_char(' ')) return false;
            if (!out.write_double(p.y)) return false;
            if (!out.write_char(' ')) return false;
            if (!out.write_double(p.z)) return false;
            if (!out.write_char('\n')) return false;
            slot_to_idx[vh.id().slot] = idx++;  // 为每个slot内的顶点重新生成有序id
        }

        VtkCellOutput vtk_cells;
        if (!collect_vtk_cell_output(mesh, slot_to_idx, vtk_cells)) return false;

        const size_t num_cells = vtk_cells.cells.size();
        if (!out.write_string("\nCELLS ")) return false;
        if (!out.write_uint64(static_cast<std::uint64_t>(num_cells))) return false;
        if (!out.write_char(' ')) return false;
        if (!out.write_uint64(static_cast<std::uint64_t>(vtk_cells.total_size))) return false;
        if (!out.write_char('\n')) return false;

        for (const auto& c : vtk_cells.cells) {
            if (!out.write_uint64(static_cast<std::uint64_t>(c.size()))) return false;
            for (size_t v : c) {
                if (!out.write_char(' ')) return false;
                if (!out.write_uint64(static_cast<std::uint64_t>(v))) return false;
            }
            if (!out.write_char('\n')) return false;
        }

        if (!out.write_string("\nCELL_TYPES ")) return false;
        if (!out.write_uint64(static_cast<std::uint64_t>(num_cells))) return false;
        if (!out.write_char('\n')) return false;
        for (int ct : vtk_cells.cell_types) {
            if (!out.write_int(ct)) return false;
            if (!out.write_char('\n')) return false;
        }

        return out.flush() && out.ok();
    } catch (...) {
        return false;
    }
}

} // namespace SolidMesh
