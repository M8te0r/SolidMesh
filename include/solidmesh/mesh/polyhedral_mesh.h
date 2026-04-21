#pragma once
#include <vector>
#include <string>
#include <unordered_map>

#include "solidmesh/math/vector3.h"
#include "solidmesh/mesh/handles.h"
#include "solidmesh/mesh/entity_pool.h"
#include "solidmesh/mesh/cell_topology.h"
#include "solidmesh/mesh/mesh_elements.h"
#include "solidmesh/mesh/face_key.h"

namespace SolidMesh {

class PolyhedralMesh;

class VertexHandle;
class FaceHandle;
class HalfFaceHandle;
class CellHandle;

// =========================================================================
// Lightweight handle wrappers — just (mesh*, ID).
// All topology queries go through the mesh (mesh-central).
// =========================================================================

class VertexHandle {
public:
    VertexHandle() = default;
    VertexHandle(PolyhedralMesh* m, VertexID id) : mesh_(m), id_(id) {}

    bool      is_valid()    const noexcept;
    VertexID  id()          const noexcept { return id_; }

    Vector3   position()    const;
    void      set_position(const Vector3& p);

    bool      is_boundary() const;

    bool operator==(const VertexHandle& o) const noexcept { return id_ == o.id_; }
    bool operator!=(const VertexHandle& o) const noexcept { return id_ != o.id_; }
    explicit operator bool() const noexcept { return is_valid(); }

private:
    PolyhedralMesh* mesh_ = nullptr;
    VertexID       id_;
};

class FaceHandle {
public:
    FaceHandle() = default;
    FaceHandle(PolyhedralMesh* m, FaceID id) : mesh_(m), id_(id) {}

    bool    is_valid()    const noexcept;
    FaceID  id()          const noexcept { return id_; }

    bool    is_boundary() const;

    std::vector<VertexHandle>                      vertices()  const;
    // hf[0] is always valid; hf[1] is invalid for boundary faces
    std::pair<HalfFaceHandle, HalfFaceHandle>      halffaces() const;

    bool operator==(const FaceHandle& o) const noexcept { return id_ == o.id_; }
    bool operator!=(const FaceHandle& o) const noexcept { return id_ != o.id_; }
    explicit operator bool() const noexcept { return is_valid(); }

private:
    PolyhedralMesh* mesh_ = nullptr;
    FaceID         id_;
};

class HalfFaceHandle {
public:
    HalfFaceHandle() = default;
    HalfFaceHandle(PolyhedralMesh* m, HalfFaceID id) : mesh_(m), id_(id) {}

    bool          is_valid()    const noexcept;
    HalfFaceID    id()          const noexcept { return id_; }

    bool          is_boundary() const;
    CellHandle    cell()        const;
    FaceHandle    face()        const;
    HalfFaceHandle opposite()  const;

    std::vector<VertexHandle> vertices() const;

    bool operator==(const HalfFaceHandle& o) const noexcept { return id_ == o.id_; }
    bool operator!=(const HalfFaceHandle& o) const noexcept { return id_ != o.id_; }
    explicit operator bool() const noexcept { return is_valid(); }

private:
    PolyhedralMesh* mesh_ = nullptr;
    HalfFaceID     id_;
};

class CellHandle {
public:
    CellHandle() = default;
    CellHandle(PolyhedralMesh* m, CellID id) : mesh_(m), id_(id) {}

    bool      is_valid()         const noexcept;
    CellID    id()               const noexcept { return id_; }
    CellType  type()             const;

    bool      is_boundary()      const;

    std::vector<VertexHandle>   vertices()       const;
    std::vector<HalfFaceHandle> halffaces()      const;
    std::vector<CellHandle>     adjacent_cells() const;

    bool operator==(const CellHandle& o) const noexcept { return id_ == o.id_; }
    bool operator!=(const CellHandle& o) const noexcept { return id_ != o.id_; }
    explicit operator bool() const noexcept { return is_valid(); }

private:
    PolyhedralMesh* mesh_ = nullptr;
    CellID         id_;
};

// =========================================================================
// Validation report
// =========================================================================

struct ValidationIssue {
    enum class Type {
        InvalidVertexSeed,
        FaceZeroHalfFaces,
        HalfFaceBackRefBroken,
        CellVertexCountMismatch,
        CellHalfFaceCountMismatch,
        CellHalfFaceBackRefBroken,
        NonManifoldFace,
        FaceMapInconsistency,
        VertexAdjInconsistency,
    };
    Type        type;
    std::string message;
};

struct ValidationReport {
    bool                          ok = true;
    std::vector<ValidationIssue> issues;
};

// =========================================================================
// Range helpers — returned by mesh.cells() / .halffaces() / .vertices() / .faces()
// =========================================================================

template<typename T, typename Handle, typename ID>
class HandleRange {
public:
    HandleRange(PolyhedralMesh* mesh, const EntityPool<T, ID>* pool)
        : mesh_(mesh), pool_(pool) {}

    struct iterator {
        PolyhedralMesh*                              mesh;
        typename EntityPool<T, ID>::IDIterator      it;

        Handle operator*() const { return Handle(mesh, *it); }
        iterator& operator++() { ++it; return *this; }
        bool operator!=(const iterator& o) const { return it != o.it; }
    };

    iterator begin() const { return {mesh_, pool_->ids().begin()}; }
    iterator end()   const { return {mesh_, pool_->ids().end()}; }

private:
    PolyhedralMesh*              mesh_;
    const EntityPool<T, ID>*    pool_;
};

// =========================================================================
// PolyhedralMesh
// =========================================================================

class PolyhedralMesh {
public:
    PolyhedralMesh()  = default;
    ~PolyhedralMesh() = default;

    PolyhedralMesh(const PolyhedralMesh&)            = delete;
    PolyhedralMesh& operator=(const PolyhedralMesh&) = delete;
    PolyhedralMesh(PolyhedralMesh&&)                 = default;
    PolyhedralMesh& operator=(PolyhedralMesh&&)      = default;

    // ---- construction ---------------------------------------------------

    VertexHandle add_vertex(const Vector3& position);

    // verts must match the expected count for the given CellType.
    // Returns invalid handle if the input is malformed or would create a
    // non-manifold face (a face already shared by two cells).
    CellHandle add_cell(CellType type, const std::vector<VertexHandle>& verts);

    // ---- deletion -------------------------------------------------------

    // Removes the cell and cleans up its halffaces/faces.
    // Vertices are NOT deleted; use delete_isolated_vertex afterwards.
    bool delete_cell(CellHandle c);

    // Only succeeds if the vertex has no incident cells.
    bool delete_isolated_vertex(VertexHandle v);

    // ---- validity -------------------------------------------------------

    bool is_handle_valid(VertexHandle   v)  const noexcept;
    bool is_handle_valid(FaceHandle     f)  const noexcept;
    bool is_handle_valid(HalfFaceHandle hf) const noexcept;
    bool is_handle_valid(CellHandle     c)  const noexcept;

    // ---- boundary queries -----------------------------------------------

    bool is_boundary(VertexHandle   v)  const;
    bool is_boundary(FaceHandle     f)  const;
    bool is_boundary(HalfFaceHandle hf) const;
    bool is_boundary(CellHandle     c)  const;

    // ---- element counts -------------------------------------------------

    size_t num_vertices()  const noexcept { return vertices_.size(); }
    size_t num_faces()     const noexcept { return faces_.size(); }
    size_t num_halffaces() const noexcept { return halffaces_.size(); }
    size_t num_cells()     const noexcept { return cells_.size(); }

    // ---- range iteration ------------------------------------------------

    using VertexRangeT   = HandleRange<Vertex,   VertexHandle,   VertexID>;
    using FaceRangeT     = HandleRange<Face,      FaceHandle,     FaceID>;
    using HalfFaceRangeT = HandleRange<HalfFace,  HalfFaceHandle, HalfFaceID>;
    using CellRangeT     = HandleRange<Cell,       CellHandle,     CellID>;

    VertexRangeT   vertices()  { return {this, &vertices_}; }
    FaceRangeT     faces()     { return {this, &faces_}; }
    HalfFaceRangeT halffaces() { return {this, &halffaces_}; }
    CellRangeT     cells()     { return {this, &cells_}; }

    VertexRangeT   vertices()  const { return {const_cast<PolyhedralMesh*>(this), &vertices_}; }
    FaceRangeT     faces()     const { return {const_cast<PolyhedralMesh*>(this), &faces_}; }
    HalfFaceRangeT halffaces() const { return {const_cast<PolyhedralMesh*>(this), &halffaces_}; }
    CellRangeT     cells()     const { return {const_cast<PolyhedralMesh*>(this), &cells_}; }

    // ---- index-based access (TBB-friendly) ------------------------------
    // Access by dense index [0, num_X()). Index is NOT stable across deletions.

    VertexHandle   vertex_at(size_t i)   const;
    FaceHandle     face_at(size_t i)     const;
    HalfFaceHandle halfface_at(size_t i) const;
    CellHandle     cell_at(size_t i)     const;

    // ---- mesh-centric topology queries ----------------------------------

    std::vector<VertexHandle>   cell_vertices(CellHandle c)   const;
    std::vector<HalfFaceHandle> cell_halffaces(CellHandle c)  const;
    std::vector<CellHandle>     cell_cells(CellHandle c)      const;

    std::vector<VertexHandle>   halfface_vertices(HalfFaceHandle hf) const;
    HalfFaceHandle              halfface_opposite(HalfFaceHandle hf) const;
    CellHandle                  halfface_cell(HalfFaceHandle hf)     const;
    FaceHandle                  halfface_face(HalfFaceHandle hf)     const;

    std::vector<VertexHandle>                      face_vertices(FaceHandle f)  const;
    std::pair<HalfFaceHandle, HalfFaceHandle>      face_halffaces(FaceHandle f) const;

    // vertex star queries — O(degree), backed by vertex_cell_adj_
    std::vector<CellHandle>     vertex_cells(VertexHandle v)     const;
    std::vector<HalfFaceHandle> vertex_halffaces(VertexHandle v) const;

    // edge export — edges derived from cell topology, no EdgeID stored
    // Each edge appears once per cell (not deduplicated across cells).
    std::vector<std::pair<VertexHandle, VertexHandle>> cell_edges(CellHandle c) const;

    std::vector<CellHandle> edge_cells(VertexHandle v0, VertexHandle v1) const;
    std::vector<CellHandle> edge_cells_ccw(VertexHandle v0, VertexHandle v1) const;

    // ---- copy / duplicate -----------------------------------------------

    // Returns a deep copy of this mesh. All topology is rebuilt via add_vertex /
    // add_cell so the new mesh is fully self-consistent. Vertex and cell flags
    // are preserved; face/halfface flags are not (they are derived from topology).
    PolyhedralMesh duplicate() const;

    // ---- validation -----------------------------------------------------

    ValidationReport validate() const;

    // ---- internal pool access (used by handle wrappers) -----------------

    EntityPool<Vertex,   VertexID>&   vertex_pool()   { return vertices_; }
    EntityPool<Face,     FaceID>&     face_pool()     { return faces_; }
    EntityPool<HalfFace, HalfFaceID>& halfface_pool() { return halffaces_; }
    EntityPool<Cell,     CellID>&     cell_pool()     { return cells_; }

    const EntityPool<Vertex,   VertexID>&   vertex_pool()   const { return vertices_; }
    const EntityPool<Face,     FaceID>&     face_pool()     const { return faces_; }
    const EntityPool<HalfFace, HalfFaceID>& halfface_pool() const { return halffaces_; }
    const EntityPool<Cell,     CellID>&     cell_pool()     const { return cells_; }

private:
    EntityPool<Vertex,   VertexID>    vertices_;
    EntityPool<Face,     FaceID>      faces_;
    EntityPool<HalfFace, HalfFaceID>  halffaces_;
    EntityPool<Cell,     CellID>      cells_;

    std::unordered_map<FaceKey, FaceID, FaceKeyHash> face_map_;

    // Mesh-layer adjacency: vertex slot -> incident cell IDs.
    // Maintained by add_cell / delete_cell. Enables O(degree) vertex star queries.
    std::unordered_map<uint32_t, std::vector<CellID>> vertex_cell_adj_;

    void repair_vertex_seed(VertexID vid);
};

// =========================================================================
// Handle wrapper method bodies (need full PolyhedralMesh definition)
// =========================================================================

// VertexHandle
inline bool VertexHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_handle_valid(*this);
}
inline Vector3 VertexHandle::position() const {
    return mesh_->vertex_pool().get(id_).position;
}
inline void VertexHandle::set_position(const Vector3& p) {
    mesh_->vertex_pool().get(id_).position = p;
}
inline bool VertexHandle::is_boundary() const {
    return mesh_->is_boundary(*this);
}

// FaceHandle
inline bool FaceHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_handle_valid(*this);
}
inline bool FaceHandle::is_boundary() const {
    return mesh_->is_boundary(*this);
}
inline std::vector<VertexHandle> FaceHandle::vertices() const {
    return mesh_->face_vertices(*this);
}
inline std::pair<HalfFaceHandle, HalfFaceHandle> FaceHandle::halffaces() const {
    return mesh_->face_halffaces(*this);
}

// HalfFaceHandle
inline bool HalfFaceHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_handle_valid(*this);
}
inline bool HalfFaceHandle::is_boundary() const {
    return mesh_->is_boundary(*this);
}
inline CellHandle HalfFaceHandle::cell() const {
    return mesh_->halfface_cell(*this);
}
inline FaceHandle HalfFaceHandle::face() const {
    return mesh_->halfface_face(*this);
}
inline HalfFaceHandle HalfFaceHandle::opposite() const {
    return mesh_->halfface_opposite(*this);
}
inline std::vector<VertexHandle> HalfFaceHandle::vertices() const {
    return mesh_->halfface_vertices(*this);
}

// CellHandle
inline bool CellHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_handle_valid(*this);
}
inline CellType CellHandle::type() const {
    return mesh_->cell_pool().get(id_).type;
}
inline bool CellHandle::is_boundary() const {
    return mesh_->is_boundary(*this);
}
inline std::vector<VertexHandle> CellHandle::vertices() const {
    return mesh_->cell_vertices(*this);
}
inline std::vector<HalfFaceHandle> CellHandle::halffaces() const {
    return mesh_->cell_halffaces(*this);
}
inline std::vector<CellHandle> CellHandle::adjacent_cells() const {
    return mesh_->cell_cells(*this);
}

// ---- index-based access -------------------------------------------------
inline VertexHandle PolyhedralMesh::vertex_at(size_t i) const {
    return VertexHandle(const_cast<PolyhedralMesh*>(this), vertices_.id_at(i));
}
inline FaceHandle PolyhedralMesh::face_at(size_t i) const {
    return FaceHandle(const_cast<PolyhedralMesh*>(this), faces_.id_at(i));
}
inline HalfFaceHandle PolyhedralMesh::halfface_at(size_t i) const {
    return HalfFaceHandle(const_cast<PolyhedralMesh*>(this), halffaces_.id_at(i));
}
inline CellHandle PolyhedralMesh::cell_at(size_t i) const {
    return CellHandle(const_cast<PolyhedralMesh*>(this), cells_.id_at(i));
}

} // namespace SolidMesh
