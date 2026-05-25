#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace SolidMesh
{

    // MeshIO: read/write mesh files.
    //
    // Currently supports:
    //   - VTK Legacy ASCII Unstructured Grid (.vtk)
    //     Cell types handled: 10 (Tet), 12 (Hex), 13 (Prism), 14 (Pyramid)
    //     Unsupported cell types (e.g. lines, triangles) are silently skipped.
    //
    // Usage:
    class MeshIO
    {
    public:
        struct VtkRawData 
        {
            std::vector<std::vector<double>> vertices;
            std::vector<std::vector<uint32_t>> cells;
            std::vector<int> cell_types;
        };

    public:
        static bool read_vtk(const std::string &filename, VtkRawData &out_data);

        static bool write_vtk(const std::string &filename, const VtkRawData &data);
    };

} // namespace SolidMesh
