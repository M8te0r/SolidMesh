#pragma once

#include "polyhedral_mesh.h"
#include <string>

namespace SolidMesh {

// MeshIO: read/write mesh files.
//
// Currently supports:
//   - VTK Legacy ASCII Unstructured Grid (.vtk)
//     Cell types handled: 10 (Tet), 12 (Hex), 13 (Prism), 14 (Pyramid)
//     Unsupported cell types (e.g. lines, triangles) are silently skipped.
//
// Usage:
//   PolyhedralMesh mesh;
//   bool ok = MeshIO::read_vtk("path/to/file.vtk", mesh);
class MeshIO {
public:
    static bool read_vtk_fast(const std::string& path, PolyhedralMesh& mesh);
    static bool write_vtk_fast(const std::string& path, const PolyhedralMesh& mesh);
};

} // namespace SolidMesh
