#include "mesh_io.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <cctype>

namespace SolidMesh {

// VTK cell type codes relevant to volumetric meshes
static constexpr int VTK_TETRA   = 10;
static constexpr int VTK_HEXAHEDRON = 12;
static constexpr int VTK_PRISM   = 13;  // VTK_WEDGE
static constexpr int VTK_PYRAMID = 14;

// VTK hex vertex order differs from our canonical order.
// VTK hex: bottom face (v0,v1,v2,v3) CCW from above, top face (v4,v5,v6,v7).
// Our Hex: same convention — no reordering needed.
//
// VTK prism (wedge): bottom tri (v0,v1,v2), top tri (v3,v4,v5).
// Our Prism: same convention — no reordering needed.
//
// VTK pyramid: base quad (v0,v1,v2,v3), apex v4.
// Our Pyramid: same convention — no reordering needed.
//
// VTK tet: (v0,v1,v2,v3).
// Our Tet: same convention — no reordering needed.

static std::string to_upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool MeshIO::read_vtk(const std::string& path, Polyhedra& mesh) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;

    // ---- Header: 3 fixed lines ----
    // Line 1: "# vtk DataFile Version X.Y"
    // Line 2: title (ignored)
    // Line 3: "ASCII" or "BINARY"
    std::getline(f, line);  // version
    std::getline(f, line);  // title
    std::getline(f, line);  // ASCII/BINARY
    if (to_upper(line).find("ASCII") == std::string::npos) return false;

    // Line 4: "DATASET UNSTRUCTURED_GRID"
    std::getline(f, line);
    if (to_upper(line).find("UNSTRUCTURED_GRID") == std::string::npos) return false;

    std::vector<Vector3> points;
    std::vector<std::vector<int>> cell_conn;  // raw connectivity lists
    std::vector<int> cell_types;

    // ---- Parse sections ----
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string keyword;
        ss >> keyword;
        // Skip blank / whitespace-only lines (also handles \r\n endings)
        if (keyword.empty()) continue;
        keyword = to_upper(keyword);

        if (keyword == "POINTS") {
            uint32_t n_points;
            std::string dtype;
            ss >> n_points >> dtype;
            points.reserve(n_points);

            for (uint32_t i = 0; i < n_points; ++i) {
                double x, y, z;
                f >> x >> y >> z;
                points.push_back({x, y, z});
            }
            // consume rest of last number's line
            std::getline(f, line);

        } else if (keyword == "CELLS") {
            uint32_t n_cells, total_ints;
            ss >> n_cells >> total_ints;
            cell_conn.reserve(n_cells);

            for (uint32_t i = 0; i < n_cells; ++i) {
                int n;
                f >> n;
                std::vector<int> conn(n);
                for (int k = 0; k < n; ++k) f >> conn[k];
                cell_conn.push_back(std::move(conn));
            }
            std::getline(f, line);

        } else if (keyword == "CELL_TYPES") {
            uint32_t n;
            ss >> n;
            cell_types.reserve(n);
            for (uint32_t i = 0; i < n; ++i) {
                int t;
                f >> t;
                cell_types.push_back(t);
            }
            std::getline(f, line);

        } else {
            // POINT_DATA, CELL_DATA, etc. — not needed, stop parsing
            break;
        }
    }

    if (points.empty()) return false;
    if (cell_conn.size() != cell_types.size()) return false;

    // ---- Populate mesh ----
    mesh.vertices_map().reserve(static_cast<uint32_t>(points.size()));
    // VTK point index i -> our VertexID
    std::vector<VertexID> vid(points.size());
    for (size_t i = 0; i < points.size(); ++i)
        vid[i] = mesh.add_vertex(points[i]);

    for (size_t i = 0; i < cell_conn.size(); ++i) {
        const auto& conn = cell_conn[i];
        int type = cell_types[i];

        switch (type) {
            case VTK_TETRA: {
                if (conn.size() < 4) break;
                mesh.add_tet(vid[conn[0]], vid[conn[1]], vid[conn[2]], vid[conn[3]]);
                break;
            }
            case VTK_HEXAHEDRON: {
                if (conn.size() < 8) break;
                mesh.add_hex(vid[conn[0]], vid[conn[1]], vid[conn[2]], vid[conn[3]],
                             vid[conn[4]], vid[conn[5]], vid[conn[6]], vid[conn[7]]);
                break;
            }
            case VTK_PRISM: {
                if (conn.size() < 6) break;
                mesh.add_prism(vid[conn[0]], vid[conn[1]], vid[conn[2]],
                               vid[conn[3]], vid[conn[4]], vid[conn[5]]);
                break;
            }
            case VTK_PYRAMID: {
                if (conn.size() < 5) break;
                mesh.add_pyramid(vid[conn[0]], vid[conn[1]], vid[conn[2]],
                                 vid[conn[3]], vid[conn[4]]);
                break;
            }
            default:
                // Unsupported cell type (lines, triangles, etc.) — skip
                break;
        }
    }

    return true;
}

} // namespace SolidMesh
