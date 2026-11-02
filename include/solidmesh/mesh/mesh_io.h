#pragma once

#include "mesh.h"
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
//   Mesh mesh;
//   bool ok = MeshIO::read_vtk("path/to/file.vtk", mesh);
class MeshIO {
public:
    // Atomic on read failure. Unsupported non-volume cell types are skipped.
    static bool read_vtk(const std::string& path, Mesh& mesh);
    // Reconstructs a VTK-compatible positive local numbering from oriented face rings.
    // Equivalent numbering may differ after a custom face-order insertion.
    static bool write_vtk(const std::string& path, const Mesh& mesh);
};

} // namespace SolidMesh
