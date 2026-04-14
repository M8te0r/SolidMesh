#include <iostream>
#include <cassert>
#include "solidmesh/mesh/polyhedra_mesh.h"
#include "solidmesh/mesh/edge_iterator.h"
#include "solidmesh/topology_operations.h"
#include "solidmesh/mesh/mesh_io.h"

using namespace SolidMesh;

void SimpleTwoMeshTest()
{
    Polyhedra mesh;

    // Build a simple 2-tet mesh sharing one face.
    //
    //       v3
    //      /|\
    //     / | \
    //    /  |  \
    //   v0--v2--v1
    //    \  |  /
    //     \ | /
    //      \|/
    //       v4
    //
    // Tet A: v0, v1, v2, v3  (top)
    // Tet B: v0, v2, v1, v4  (bottom, shares face v0-v1-v2 with A)

    VertexID v0 = mesh.add_vertex(Vector3{0.0, 0.0, 0.0});
    VertexID v1 = mesh.add_vertex(Vector3{2.0, 0.0, 0.0});
    VertexID v2 = mesh.add_vertex(Vector3{1.0, 1.0, 0.0});
    VertexID v3 = mesh.add_vertex(Vector3{1.0, 0.5, 1.5});  // apex top
    VertexID v4 = mesh.add_vertex(Vector3{1.0, 0.5, -1.5}); // apex bottom

    CellID tetA = mesh.add_tet(v0, v1, v2, v3);
    CellID tetB = mesh.add_tet(v0, v2, v1, v4);

    std::cout << "=== Initial mesh ===\n";
    std::cout << "Vertices: " << mesh.num_vertices() << "\n";
    std::cout << "Faces:    " << mesh.num_faces() << "\n";
    std::cout << "Cells:    " << mesh.num_cells() << "\n";
    assert(mesh.num_vertices() == 5);
    assert(mesh.num_cells() == 2);
    assert(mesh.check_validity());
    std::cout << "check_validity: PASS\n\n";

    // Test edge_cells: edge v0-v1 should be shared by both tets
    {
        std::vector<CellID> ring;
        mesh.edge_cells(v0, v1, ring);
        std::cout << "edge_cells(v0,v1): " << ring.size() << " cells\n";
        assert(ring.size() == 2);
        std::cout << "edge_cells: PASS\n\n";
    }

    // Test swap23: replace 2 tets with 3 tets
    {
        // Find the shared interior face (the one with two owners)
        FaceID shared_fid = INVALID_ID;
        for (uint32_t di = 0; di < mesh.faces_map().size(); ++di)
        {
            FaceID fid = mesh.faces_map().id_of(di);
            const Face &f = mesh.face(fid);
            if (f.cell[0] != INVALID_ID && f.cell[1] != INVALID_ID)
            {
                shared_fid = fid;
                break;
            }
        }
        assert(shared_fid != INVALID_ID);

        std::array<CellID, 3> new_cells;
        bool ok = swap23(mesh, make_hf(shared_fid, 0), new_cells);
        std::cout << "swap23 result: " << (ok ? "OK" : "FAIL") << "\n";
        assert(ok);
        std::cout << "Cells after swap23: " << mesh.num_cells() << "\n";
        assert(mesh.num_cells() == 3);
        assert(mesh.check_validity());
        std::cout << "check_validity after swap23: PASS\n\n";
    }

    // Test edge_split on edge v3-v4 (shared by all 3 new tets)
    {
        VertexID new_v;
        std::vector<CellID> new_cells;
        bool ok = edge_split(mesh, v3, v4, new_v, new_cells);
        std::cout << "edge_split(v3,v4) result: " << (ok ? "OK" : "FAIL") << "\n";
        assert(ok);
        std::cout << "Cells after edge_split: " << mesh.num_cells() << "\n";
        assert(mesh.check_validity());
        std::cout << "check_validity after edge_split: PASS\n\n";
    }

    std::cout << "=== All smoke tests passed ===\n";
}

void VtkReadTest(const std::string& path) {
    std::cout << "\n=== VTK read: " << path << " ===\n";
    Polyhedra mesh;
    bool ok = MeshIO::read_vtk(path, mesh);
    if (!ok) {
        std::cout << "read_vtk FAILED\n";
        return;
    }
    std::cout << "Vertices: " << mesh.num_vertices() << "\n";
    std::cout << "Faces:    " << mesh.num_faces()    << "\n";
    std::cout << "Cells:    " << mesh.num_cells()    << "\n";
    assert(mesh.check_validity());
    std::cout << "check_validity: PASS\n";
}

int main()
{
    SimpleTwoMeshTest();
    VtkReadTest("assets/Ankle_1.vtk");
    VtkReadTest("assets/bpgc.vtk");
    return 0;
}
