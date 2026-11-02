#include "solidmesh/mesh/mesh_io.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <string>
#include <unordered_map>
#include <vector>

namespace SolidMesh {
namespace {
template<std::size_t V, std::size_t F, std::size_t E>
bool vtk_vertices(const Mesh& mesh, const Cell& cell,
                  const std::array<std::array<int,E>,F>& reference,
                  std::vector<VertexHandle>& out) {
    std::array<VertexHandle,V> unique{};
    std::size_t vertex_count=0,face_count=0;
    std::array<std::vector<int>,F> rings;
    for(auto h:cell.halffaces) {
        if(h.empty()) continue;
        if(face_count==F) return false;
        const auto* hf=mesh.halfface(h);
        if(!hf) return false;
        for(auto e:hf->dedges) {
            if(e.empty()) continue;
            const auto* edge=mesh.dedge(e);
            if(!edge||!mesh.vertex(edge->v0)) return false;
            std::size_t i=0;
            while(i<vertex_count&&unique[i]!=edge->v0) ++i;
            if(i==vertex_count) {
                if(vertex_count==V) return false;
                unique[vertex_count++]=edge->v0;
            }
            rings[face_count].push_back(static_cast<int>(i));
        }
        ++face_count;
    }
    if(vertex_count!=V||face_count!=F) return false;
    std::array<std::vector<int>,F> expected;
    for(std::size_t f=0;f<F;++f) for(int i:reference[f]) if(i>=0) expected[f].push_back(i);
    // Match an oriented anchor face, then permute the remaining vertices. For the
    // standard insertion templates the first mapping succeeds, including HEX.
    for(const auto& anchor:rings) {
        if(anchor.size()!=expected[0].size()) continue;
        for(std::size_t shift=0;shift<anchor.size();++shift) {
            std::array<int,V> mapping;mapping.fill(-1);
            std::array<bool,V> used{};
            for(std::size_t j=0;j<anchor.size();++j) {
                mapping[expected[0][j]]=anchor[(shift+j)%anchor.size()];
                used[anchor[(shift+j)%anchor.size()]]=true;
            }
            std::vector<int> free,missing;
            for(std::size_t i=0;i<V;++i) {
                if(!used[i]) free.push_back(static_cast<int>(i));
                if(mapping[i]<0) missing.push_back(static_cast<int>(i));
            }
            do {
                for(std::size_t i=0;i<free.size();++i) mapping[missing[i]]=free[i];
                bool match=true;
                std::array<bool,F> matched{};
                for(const auto& target:expected) {
                    bool found=false;
                    for(std::size_t f=0;f<F&&!found;++f) {
                        const auto& actual=rings[f];
                        if(matched[f]||actual.size()!=target.size()) continue;
                        for(std::size_t start=0;start<actual.size();++start) {
                            bool equal=true;
                            for(std::size_t k=0;k<target.size();++k)
                                equal=equal&&mapping[target[k]]==actual[(start+k)%actual.size()];
                            if(equal) {matched[f]=true;found=true;break;}
                        }
                    }
                    if(!found) {match=false;break;}
                }
                if(match) {
                    out.clear();
                    for(auto i:mapping) out.push_back(unique[i]);
                    return true;
                }
            } while(std::next_permutation(free.begin(),free.end()));
        }
    }
    return false;
}
bool ordered_vertices(const Mesh& mesh,const Cell& cell,std::vector<VertexHandle>& out,int& type) {
    switch(cell.type) {
    case CellType::TET:type=10;return vtk_vertices<4>(mesh,cell,CellOrderTemplate::tet_faces,out);
    case CellType::HEX:type=12;return vtk_vertices<8>(mesh,cell,CellOrderTemplate::hex_faces,out);
    case CellType::PYRAMID:type=14;return vtk_vertices<5>(mesh,cell,CellOrderTemplate::pyramid_faces,out);
    case CellType::PRISM:type=13;return vtk_vertices<6>(mesh,cell,CellOrderTemplate::prism_faces,out);
    default:return false;
    }
}
}

bool MeshIO::read_vtk(const std::string& path,Mesh& mesh) {
    try {
        std::ifstream in(path);in.imbue(std::locale::classic());
        if(!in) return false;
        std::string line;
        if(!std::getline(in,line)||line.find("# vtk DataFile Version")!=0) return false;
        if(!std::getline(in,line)||!std::getline(in,line)) return false;
        if(line!="ASCII"&&line!="ascii"&&line!="ASCII\r"&&line!="ascii\r") return false;
        if(!std::getline(in,line)||line.find("DATASET UNSTRUCTURED_GRID")!=0) return false;
        std::vector<Vector3> points;
        std::vector<std::vector<std::uint64_t>> cells;
        std::vector<int> types;
        bool have_points=false,have_cells=false,have_types=false;
        std::string token;
        while(in>>token) {
            if(token=="POINTS") {
                std::int64_t count=0;std::string scalar;
                if(!(in>>count>>scalar)||count<0||std::uint64_t(count)>=std::numeric_limits<std::uint32_t>::max()) return false;
                points.resize(static_cast<std::size_t>(count));
                for(auto& p:points) if(!(in>>p.x>>p.y>>p.z)) return false;
                have_points=true;
            } else if(token=="CELLS") {
                std::int64_t count=0,total=0;
                if(!(in>>count>>total)||count<0||total<count||std::uint64_t(count)>=std::numeric_limits<std::uint32_t>::max()) return false;
                cells.resize(static_cast<std::size_t>(count));
                std::int64_t consumed=0;
                for(auto& c:cells) {
                    std::int64_t n=0;
                    if(!(in>>n)||n<0||consumed>=total||n>total-consumed-1) return false;
                    consumed+=n+1;c.resize(static_cast<std::size_t>(n));
                    for(auto& index:c) if(!(in>>index)) return false;
                }
                if(consumed!=total) return false;
                have_cells=true;
            } else if(token=="CELL_TYPES") {
                std::int64_t count=0;
                if(!(in>>count)||count<0||std::uint64_t(count)>=std::numeric_limits<std::uint32_t>::max()) return false;
                types.resize(static_cast<std::size_t>(count));
                for(auto& t:types) if(!(in>>t)) return false;
                have_types=true;
                if(have_points&&have_cells) break;
            }
        }
        if(!have_points||!have_cells||!have_types||cells.size()!=types.size()) return false;
        Mesh loaded;std::vector<VertexHandle> vertices;vertices.reserve(points.size());
        for(auto p:points) {
            auto v=loaded.add_isolated_vertex(p.x,p.y,p.z);
            if(v.empty()) return false;
            vertices.push_back(v);
        }
        std::array<VertexHandle,8> v{};
        for(std::size_t i=0;i<cells.size();++i) {
            std::size_t expected=0;
            switch(types[i]) {case 10:expected=4;break;case 12:expected=8;break;case 13:expected=6;break;case 14:expected=5;break;default:continue;}
            if(cells[i].size()!=expected) return false;
            for(std::size_t j=0;j<expected;++j) {
                if(cells[i][j]>=vertices.size()) return false;
                v[j]=vertices[static_cast<std::size_t>(cells[i][j])];
            }
            CellHandle c;
            switch(types[i]) {
            case 10:c=loaded.add_tet({v[0],v[1],v[2],v[3]});break;
            case 12:c=loaded.add_hex(v);break;
            case 13:c=loaded.add_prism({v[0],v[1],v[2],v[3],v[4],v[5]});break;
            case 14:c=loaded.add_pyramid({v[0],v[1],v[2],v[3],v[4]});break;
            }
            if(c.empty()) return false;
        }
        mesh=std::move(loaded);return true;
    } catch(...) {return false;}
}

bool MeshIO::write_vtk(const std::string& path,const Mesh& mesh) {
    try {
        // No editing while exporting: EntityPool element pointers are stable for this pass.
        std::unordered_map<const Vertex*,std::size_t> indices;indices.reserve(mesh.num_vertices());
        std::vector<std::vector<std::size_t>> cells;cells.reserve(mesh.num_cells());
        std::vector<int> types;types.reserve(mesh.num_cells());
        std::size_t index=0,total=0;
        for(auto v:mesh.vertices()) indices.emplace(mesh.vertex(v),index++);
        std::vector<VertexHandle> vertices;vertices.reserve(8);
        for(auto c:mesh.cells()) {
            int type=0;
            if(!ordered_vertices(mesh,*mesh.cell(c),vertices,type)) return false;
            std::vector<std::size_t> conn;conn.reserve(vertices.size());
            for(auto v:vertices) conn.push_back(indices.at(mesh.vertex(v)));
            total+=conn.size()+1;cells.push_back(std::move(conn));types.push_back(type);
        }
        std::ofstream out(path);out.imbue(std::locale::classic());
        if(!out) return false;
        out<<std::setprecision(std::numeric_limits<Real>::max_digits10);
        out<<"# vtk DataFile Version 3.0\nSolidMesh export\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS "<<mesh.num_vertices()<<" double\n";
        for(auto v:mesh.vertices()) {
            const auto& p=mesh.vertex(v)->pos;out<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
        }
        out<<"\nCELLS "<<cells.size()<<' '<<total<<'\n';
        for(const auto& c:cells) {out<<c.size();for(auto i:c) out<<' '<<i;out<<'\n';}
        out<<"\nCELL_TYPES "<<types.size()<<'\n';for(auto type:types) out<<type<<'\n';
        out.flush();return bool(out);
    } catch(...) {return false;}
}
}
