#pragma once

#include "polyhedra_mesh.h"
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
//   PolyhedraMesh mesh;
//   bool ok = MeshIO::read_vtk("path/to/file.vtk", mesh);
class MeshIO {
public:
    static bool read_vtk(const std::string& path, PolyhedraMesh& mesh);
};

} // namespace SolidMesh
