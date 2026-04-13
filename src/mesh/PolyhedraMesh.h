#pragma once

#include "Handles.h"
#include "SlotMap.h"
#include "MeshEntities.h"
#include "CellTopology.h"
#include "linear-algebra/vector3.h"

#include <array>
#include <vector>
#include <unordered_map>
#include <functional>

namespace SolidMesh {

// FaceKey: sorted vertex IDs of a face (tri: [v0,v1,v2,INVALID_ID], quad: [v0,v1,v2,v3]).
// Used to identify shared faces between adjacent cells.
using FaceKey = std::array<VertexID, 4>;

struct FaceKeyHash {
    size_t operator()(const FaceKey& k) const noexcept {
        // FNV-1a style mix
        size_t h = 2166136261u;
        for (auto v : k) {
            h ^= static_cast<size_t>(v);
            h *= 16777619u;
        }
        return h;
    }
};

// HybridMesh: half-face based hybrid volumetric mesh.
//
// Supports Tet, Hex, Pyramid, and Prism cells.
// Half-faces are encoded as HalfFaceID = FaceID*2 + side (see Handles.h).
// All entities are managed by SlotMaps for O(1) insert/erase.
class Polyhedra {
public:
    Polyhedra() = default;

    // ---- Vertex operations ----
    VertexID add_vertex(const Vector3& pos);
    VertexID add_vertex(const Vector3& pos, const Vector3& normal);
    void     remove_vertex(VertexID v);  // asserts no incident cells

    Vertex&       vertex(VertexID v)       { return vertices_[v]; }
    const Vertex& vertex(VertexID v) const { return vertices_[v]; }

    // ---- Cell operations ----
    // verts must be in canonical order per CellTopology tables.
    CellID add_cell(CellType type, const VertexID* verts, uint32_t vcount);

    CellID add_tet(VertexID v0, VertexID v1, VertexID v2, VertexID v3);
    CellID add_hex(VertexID v0, VertexID v1, VertexID v2, VertexID v3,
                   VertexID v4, VertexID v5, VertexID v6, VertexID v7);
    CellID add_pyramid(VertexID v0, VertexID v1, VertexID v2, VertexID v3, VertexID v4);
    CellID add_prism(VertexID v0, VertexID v1, VertexID v2,
                     VertexID v3, VertexID v4, VertexID v5);

    void remove_cell(CellID c);

    Cell&       cell(CellID c)       { return cells_[c]; }
    const Cell& cell(CellID c) const { return cells_[c]; }

    // ---- Face access ----
    Face&       face(FaceID f)       { return faces_[f]; }
    const Face& face(FaceID f) const { return faces_[f]; }

    // ---- Counts ----
    uint32_t num_vertices() const { return vertices_.size(); }
    uint32_t num_faces()    const { return faces_.size(); }
    uint32_t num_cells()    const { return cells_.size(); }

    // ---- Adjacency queries ----

    // All cells incident to vertex v.
    const std::vector<CellID>& vertex_cells(VertexID v) const;

    // All half-face IDs owned by cell c (in canonical order).
    void cell_halffaces(CellID c, std::vector<HalfFaceID>& out) const;

    // All cells sharing edge (v0, v1), in ring order.
    // For boundary edges the ring is open (chain).
    void edge_cells(VertexID v0, VertexID v1, std::vector<CellID>& out) const;

    // All face IDs containing edge (v0, v1).
    const std::vector<FaceID>& edge_faces(VertexID v0, VertexID v1) const;

    // ---- Boundary queries ----
    bool is_boundary_face(FaceID f) const;
    bool is_boundary_halfface(HalfFaceID hf) const;
    bool is_boundary_edge(VertexID v0, VertexID v1) const;
    bool is_boundary_vertex(VertexID v) const;

    // ---- Validity ----
    // Returns true if the mesh is topologically consistent.
    bool check_validity() const;

    // ---- Dense iteration ----
    // Iterate over all live vertices/cells/faces (dense, unordered).
    // Use id_of() on the SlotMap to get the ID of each element.
    const SlotMap<Vertex>& vertices_map() const { return vertices_; }
    const SlotMap<Face>&   faces_map()    const { return faces_; }
    const SlotMap<Cell>&   cells_map()    const { return cells_; }

    SlotMap<Vertex>& vertices_map() { return vertices_; }
    SlotMap<Face>&   faces_map()    { return faces_; }
    SlotMap<Cell>&   cells_map()    { return cells_; }

private:
    SlotMap<Vertex> vertices_;
    SlotMap<Face>   faces_;
    SlotMap<Cell>   cells_;

    // Face lookup: sorted vertex key -> FaceID
    std::unordered_map<FaceKey, FaceID, FaceKeyHash> face_map_;

    // Edge lookup: EdgeKey -> list of FaceIDs containing that edge
    std::unordered_map<EdgeKey, std::vector<FaceID>> edge_faces_;

    // Vertex -> incident cells (adjacency list, indexed by sparse VertexID)
    // Grows on demand; entries for dead vertices are cleared on remove_vertex.
    std::unordered_map<VertexID, std::vector<CellID>> vcells_;

    // ---- Internal helpers ----
    FaceKey  make_face_key(const VertexID* verts, uint32_t vcount) const;
    FaceID   find_or_create_face(const VertexID* verts, uint32_t vcount,
                                 CellID owner_cell, int side);
    void     unlink_cell_from_faces(CellID c);
    void     remove_face_if_orphan(FaceID f);
    void     add_vcell(VertexID v, CellID c);
    void     remove_vcell(VertexID v, CellID c);

    static const std::vector<FaceID> EMPTY_FACE_LIST;
    static const std::vector<CellID> EMPTY_CELL_LIST;
};

} // namespace SolidMesh
