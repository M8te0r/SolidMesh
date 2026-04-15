#include <iostream>
#include <chrono>
#include "solidmesh/mesh/polyhedra_mesh.h"
#include "solidmesh/mesh/mesh_io.h"

using namespace SolidMesh;

void SimpleExample(){
    PolyhedraMesh mesh;

    // Build a simple mesh of two tetrahedra sharing one face
    //
    //       3
    //      /|\
    //     / | \
    //    /  |  \
    //   0---1---4
    //    \  |  /
    //     \ | /
    //      \|/
    //       2
    //
    // Tet A: 0,1,2,3
    // Tet B: 1,4,2,3  (shares face 1,2,3 with Tet A)

    auto v0 = mesh.add_vertex({-1,  0, -1});
    auto v1 = mesh.add_vertex({ 0,  0,  0});
    auto v2 = mesh.add_vertex({ 0, -1,  0});
    auto v3 = mesh.add_vertex({ 0,  1,  0});
    auto v4 = mesh.add_vertex({ 1,  0, -1});

    auto c0 = mesh.add_cell(CellType::Tet, {v0, v1, v2, v3});
    auto c1 = mesh.add_cell(CellType::Tet, {v1, v4, v2, v3});

    std::cout << "Mesh: "
              << mesh.num_vertices() << " vertices, "
              << mesh.num_cells()    << " cells, "
              << mesh.num_halffaces()<< " halffaces, "
              << mesh.num_faces()    << " faces\n\n";

    // --- Iterate over all cells ---
    std::cout << "=== Cells ===\n";
    for (auto cell : mesh.cells()) {
        std::cout << "  Cell (slot=" << cell.id().slot << ")"
                  << "  boundary=" << cell.is_boundary() << "\n";

        std::cout << "    vertices:";
        for (auto v : cell.vertices())
            std::cout << " " << v.id().slot;
        std::cout << "\n";

        std::cout << "    adjacent cells:";
        for (auto nb : cell.adjacent_cells())
            std::cout << " " << nb.id().slot;
        std::cout << "\n";
    }

    // --- Iterate over all halffaces ---
    std::cout << "\n=== HalfFaces ===\n";
    for (auto hf : mesh.halffaces()) {
        std::cout << "  HF (slot=" << hf.id().slot << ")"
                  << "  boundary=" << hf.is_boundary()
                  << "  cell=" << hf.cell().id().slot
                  << "\n";
    }

    // --- Iterate over all vertices ---
    std::cout << "\n=== Vertices ===\n";
    for (auto v : mesh.vertices()) {
        std::cout << "  V (slot=" << v.id().slot << ")"
                  << "  boundary=" << v.is_boundary()
                  << "  pos=(" << v.position().x << ","
                               << v.position().y << ","
                               << v.position().z << ")\n";
    }

    // --- Validate ---
    auto report = mesh.validate();
    std::cout << "\nValidation: " << (report.ok ? "OK" : "FAILED") << "\n";
    for (const auto& issue : report.issues)
        std::cout << "  Issue: " << issue.message << "\n";

    // --- Delete a cell and check boundary updates ---
    std::cout << "\n--- Deleting cell c0 ---\n";
    mesh.delete_cell(c0);
    std::cout << "After delete: "
              << mesh.num_cells() << " cells, "
              << mesh.num_halffaces() << " halffaces, "
              << mesh.num_faces() << " faces\n";

    std::cout << "Remaining cell boundary=" << c1.is_boundary() << "\n";

    auto report2 = mesh.validate();
    std::cout << "Validation after delete: " << (report2.ok ? "OK" : "FAILED") << "\n";
    for (const auto& issue : report2.issues)
        std::cout << "  Issue: " << issue.message << "\n";
}

int main() {
    std::string path="assets/bpgc.vtk";

    // start time
    auto start = std::chrono::high_resolution_clock::now();
    PolyhedraMesh mesh;
    
    if(!MeshIO::read_vtk(path,mesh)){
        printf("Read .vtk FAILED!\n");
        return 0;
    }

    // end time
    auto end = std::chrono::high_resolution_clock::now();

    // time cost, ms
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Mesh: "
              << mesh.num_vertices() << " vertices, "
              << mesh.num_cells()    << " cells, "
              << mesh.num_halffaces()<< " halffaces, "
              << mesh.num_faces()    << " faces\n";
    std::cout << "Time cost: " << duration.count() << " ms\n\n";

    return 0;
}
