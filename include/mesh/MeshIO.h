#pragma once

#include "PolyhedraMesh.h"
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
//   Polyhedra mesh;
//   bool ok = MeshIO::read_vtk("path/to/file.vtk", mesh);
class MeshIO {
public:
    // Read a VTK Legacy ASCII Unstructured Grid file into mesh.
    // Returns true on success. On failure, mesh may be partially populated.
    static bool read_vtk(const std::string& path, Polyhedra& mesh);
};

} // namespace SolidMesh
