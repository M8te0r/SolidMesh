#include <solidmesh/mesh/mesh.h>
#include <solidmesh/mesh/mesh_io.h>
#include <iostream>

int main(int argc,char** argv) {
    SolidMesh::Mesh mesh;
    if(argc>1) {
        if(!SolidMesh::MeshIO::read_vtk(argv[1],mesh)) {
            std::cerr<<"Cannot read VTK: "<<argv[1]<<'\n';return 1;
        }
    } else {
        auto a=mesh.add_isolated_vertex(0,0,0);
        auto b=mesh.add_isolated_vertex(1,0,0);
        auto c=mesh.add_isolated_vertex(0,1,0);
        auto d=mesh.add_isolated_vertex(0,0,1);
        auto e=mesh.add_isolated_vertex(0,0,-1);
        auto first=mesh.add_tet({a,b,c,d});auto second=mesh.add_tet({a,c,b,e});
        if(first.empty()||second.empty()||mesh.cell_cells(first).size()!=1) return 2;
        auto copy=mesh.copy();
        if(!copy.delete_cell(first)||!mesh.cell(first)||copy.cell(first)) return 3;
        const auto& position=mesh.vertex(a)->pos;
        std::cout<<"First position: "<<position[0]<<' '<<position[1]<<' '<<position[2]<<'\n';
    }
    std::cout<<mesh.num_vertices()<<" vertices, "<<mesh.num_dedges()<<" directed edges, "
             <<mesh.num_halffaces()<<" halffaces, "<<mesh.num_cells()<<" cells\n";
    if(argc>2&&!SolidMesh::MeshIO::write_vtk(argv[2],mesh)) return 4;
    return 0;
}
