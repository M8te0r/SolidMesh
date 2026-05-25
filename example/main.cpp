#include <iostream>
#include <chrono>
#include "solidmesh/mesh/mesh_io.h"
#include "solidmesh/mesh/mesh.h"

bool same_ids(std::vector<std::uint32_t> a, std::vector<std::uint32_t> b)
{
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    return a == b;
}

int SimpleExample(){
    std::vector<std::vector<double>> vertices = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 1.0, 1.0}};

    std::vector<std::vector<std::uint32_t>> cells = {
        {0, 1, 2, 3},
        {1, 2, 3, 4}};

    std::vector<SolidMesh::CellType> celltypes = {
        SolidMesh::CellType::Tet,
        SolidMesh::CellType::Tet};

    SolidMesh::Mesh mesh;
    mesh.build_topology(vertices, cells, celltypes);

    bool ok = true;
    auto check = [&](bool condition, const char *name) {
        std::cout << "[topology] " << name << ": " << (condition ? "ok" : "failed") << '\n';
        ok = ok && condition;
    };

    check(mesh.num_vertices() == 5, "vertex count");
    check(mesh.num_cells() == 2, "cell count");
    check(mesh.num_halffaces() == 8, "halfface count");
    check(mesh.num_faces() == 7, "face count");
    check(mesh.num_edges() == 9, "edge count");
    check(mesh.num_dedges() == 18, "directed edge count");

    int interior_faces = 0;
    for (std::uint32_t fi = 0; fi < mesh.num_faces(); ++fi)
    {
        const SolidMesh::Face &face = mesh.face_at(fi);
        if (face.hf0 != SolidMesh::invalid_id && face.hf1 != SolidMesh::invalid_id)
        {
            ++interior_faces;
            check(mesh.halfface_at(face.hf0).opposite == face.hf1, "hf0 opposite");
            check(mesh.halfface_at(face.hf1).opposite == face.hf0, "hf1 opposite");
        }
    }
    check(interior_faces == 1, "shared face count");

    check(same_ids(mesh.cell_cells(0), {1}), "cell 0 neighbor");
    check(same_ids(mesh.cell_cells(1), {0}), "cell 1 neighbor");
    check(same_ids(mesh.vertex_vertices(1), {0, 2, 3, 4}), "vertex 1 ring");

    std::uint32_t edge_12_dedge = SolidMesh::invalid_id;
    for (std::uint32_t dei = 0; dei < mesh.num_dedges(); ++dei)
    {
        const SolidMesh::DirectedEdge &dedge = mesh.dedge_at(dei);
        if (dedge.start == 1 && dedge.end == 2)
        {
            edge_12_dedge = dei;
            break;
        }
    }
    check(edge_12_dedge != SolidMesh::invalid_id, "directed edge 1 -> 2 exists");
    if (edge_12_dedge != SolidMesh::invalid_id)
    {
        check(same_ids(mesh.edge_cells_ccw(edge_12_dedge), {0, 1}), "edge 1-2 cells");
    }

    check(mesh.cell_vertices(0).size() == 4, "cell vertices span");
    check(mesh.cell_halffaces(0).size() == 4, "cell halffaces span");

    std::cout << "[topology] result: " << (ok ? "passed" : "failed") << '\n';
    return ok ? 0 : 1;
}

int main() {
    SimpleExample();
    return 0;
}

