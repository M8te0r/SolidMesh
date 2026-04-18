#include "solidmesh/mesh/mesh_io.h"

#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>

namespace SolidMesh {

bool MeshIO::read_vtk(const std::string& path, PolyhedralMesh& mesh) {
    std::ifstream in(path);
    if (!in) return false;

    // Legacy VTK header (4 lines):
    //   # vtk DataFile Version x.x
    //   <title>
    //   ASCII
    //   DATASET UNSTRUCTURED_GRID
    std::string line;
    if (!std::getline(in, line)) return false;
    if (!std::getline(in, line)) return false;
    if (!std::getline(in, line)) return false;
    if (line.find("ASCII") == std::string::npos && line.find("ascii") == std::string::npos) return false;
    if (!std::getline(in, line)) return false;
    if (line.find("UNSTRUCTURED_GRID") == std::string::npos) return false;

    std::vector<Vector3> points;
    std::vector<size_t> cell_offsets;
    std::vector<int> cell_conn;
    std::vector<int> cell_types;

    bool have_points = false;
    bool have_cells = false;
    bool have_cell_types = false;

    std::string token;
    while (in >> token) {
        if (token == "POINTS") {
            size_t num_points = 0;
            std::string scalar_type;
            if (!(in >> num_points >> scalar_type)) return false;

            points.clear();
            points.reserve(num_points);

            for (size_t i = 0; i < num_points; ++i) {
                double x = 0.0, y = 0.0, z = 0.0;
                if (!(in >> x >> y >> z)) return false;
                points.push_back(Vector3{x, y, z});
            }
            have_points = true;
            continue;
        }

        if (token == "CELLS") {
            size_t num_cells = 0, total_size = 0;
            if (!(in >> num_cells >> total_size)) return false;
            if (total_size < num_cells) return false;

            cell_offsets.clear();
            cell_offsets.resize(num_cells + 1);
            cell_conn.clear();
            cell_conn.reserve(total_size - num_cells);

            size_t consumed = 0;
            for (size_t ci = 0; ci < num_cells; ++ci) {
                int nverts = 0;
                if (!(in >> nverts)) return false;
                if (nverts < 0) return false;

                cell_offsets[ci] = cell_conn.size();
                consumed += static_cast<size_t>(nverts) + 1;

                for (int j = 0; j < nverts; ++j) {
                    int v = -1;
                    if (!(in >> v)) return false;
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
            if (!(in >> num_types)) return false;

            cell_types.clear();
            cell_types.resize(num_types);
            for (size_t i = 0; i < num_types; ++i) {
                if (!(in >> cell_types[i])) return false;
            }

            have_cell_types = true;
            if (have_points && have_cells) break;
            continue;
        }
    }

    if (!have_points || !have_cells || !have_cell_types) return false;
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
        CellType ctype;
        int expected_nverts = 0;

        switch (cell_types[ci]) {
            case 10: ctype = CellType::Tet;     expected_nverts = 4; break;
            case 12: ctype = CellType::Hex;     expected_nverts = 8; break;
            case 13: ctype = CellType::Prism;   expected_nverts = 6; break;
            case 14: ctype = CellType::Pyramid; expected_nverts = 5; break;
            default: continue; // Unsupported VTK type -> silently skip.
        }

        const size_t begin = cell_offsets[ci];
        const size_t end = cell_offsets[ci + 1];
        const size_t nv = end - begin;
        if (nv != static_cast<size_t>(expected_nverts)) return false;

        cell_verts.clear();
        for (size_t k = begin; k < end; ++k) {
            const int vtk_vid = cell_conn[k];
            if (vtk_vid < 0 || static_cast<size_t>(vtk_vid) >= vtk_vertices.size()) return false;
            cell_verts.push_back(vtk_vertices[static_cast<size_t>(vtk_vid)]);
        }

        if (!loaded.add_cell(ctype, cell_verts).is_valid()) return false;
    }

    mesh = std::move(loaded);
    return true;
}

bool MeshIO::write_vtk(const std::string& path, const PolyhedralMesh& mesh) {
    std::ofstream out(path);
    if (!out) return false;

    out << "# vtk DataFile Version 3.0\n";
    out << "SolidMesh export\n";
    out << "ASCII\n";
    out << "DATASET UNSTRUCTURED_GRID\n";

    // Write points
    std::size_t num_points = mesh.num_vertices();
    out << "POINTS " << num_points << " double\n";

    std::unordered_map<uint32_t, size_t> slot_to_idx;
    slot_to_idx.reserve(num_points);

    size_t idx = 0;
    for (auto vh : mesh.vertices()) {
        const Vector3 p = vh.position();
        out << p.x << " " << p.y << " " << p.z << "\n";
        slot_to_idx[vh.id().slot] = idx++;
    }

    // Collect cells (only supported cell types)
    std::vector<std::vector<size_t>> cells;
    std::vector<int> cell_types;
    cells.reserve(mesh.num_cells());
    cell_types.reserve(mesh.num_cells());

    for (auto ch : mesh.cells()) {
        CellType ct = ch.type();
        int vtk_type = -1;
        switch (ct) {
            case CellType::Tet:     vtk_type = 10; break;
            case CellType::Hex:     vtk_type = 12; break;
            case CellType::Prism:   vtk_type = 13; break;
            case CellType::Pyramid: vtk_type = 14; break;
            default: vtk_type = -1; break;
        }
        if (vtk_type < 0) continue; // skip unsupported

        std::vector<VertexHandle> verts = ch.vertices();
        std::vector<size_t> cverts;
        cverts.reserve(verts.size());
        bool ok = true;
        for (const auto& vh : verts) {
            auto it = slot_to_idx.find(vh.id().slot);
            if (it == slot_to_idx.end()) { ok = false; break; }
            cverts.push_back(it->second);
        }
        if (!ok) return false;
        cells.push_back(std::move(cverts));
        cell_types.push_back(vtk_type);
    }

    const size_t num_cells = cells.size();
    size_t total_size = 0;
    for (const auto& c : cells) total_size += c.size() + 1;

    out << "\nCELLS " << num_cells << " " << total_size << "\n";
    for (const auto& c : cells) {
        out << c.size();
        for (size_t v : c) out << " " << v;
        out << "\n";
    }

    out << "\nCELL_TYPES " << num_cells << "\n";
    for (int ct : cell_types) out << ct << "\n";

    return true;
}

} // namespace SolidMesh
