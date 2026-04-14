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

class PolyhedraMesh;

// Forward declarations needed for cross-references between handle types
class VertexHandle;
class HalfFaceHandle;
class CellHandle;

// =========================================================================
// Lightweight handle wrappers — just (mesh*, ID).
// All topology queries go through these.
// =========================================================================

class VertexHandle {
public:
    VertexHandle() = default;
    VertexHandle(PolyhedraMesh* m, VertexID id) : mesh_(m), id_(id) {}

    bool      is_valid()    const noexcept;
    VertexID  id()          const noexcept { return id_; }

    Vector3   position()    const;
    void      set_position(const Vector3& p);

    bool      is_boundary() const;

    bool operator==(const VertexHandle& o) const noexcept { return id_ == o.id_; }
    bool operator!=(const VertexHandle& o) const noexcept { return id_ != o.id_; }
    explicit operator bool() const noexcept { return is_valid(); }

private:
    PolyhedraMesh* mesh_ = nullptr;
    VertexID       id_;
};

class HalfFaceHandle {
public:
    HalfFaceHandle() = default;
    HalfFaceHandle(PolyhedraMesh* m, HalfFaceID id) : mesh_(m), id_(id) {}

    bool          is_valid()    const noexcept;
    HalfFaceID    id()          const noexcept { return id_; }

    bool          is_boundary() const;
    CellHandle    cell()        const;
    HalfFaceHandle opposite()  const;

    std::vector<VertexHandle> vertices() const;

    bool operator==(const HalfFaceHandle& o) const noexcept { return id_ == o.id_; }
    bool operator!=(const HalfFaceHandle& o) const noexcept { return id_ != o.id_; }
    explicit operator bool() const noexcept { return is_valid(); }

private:
    PolyhedraMesh* mesh_ = nullptr;
    HalfFaceID     id_;
};

class CellHandle {
public:
    CellHandle() = default;
    CellHandle(PolyhedraMesh* m, CellID id) : mesh_(m), id_(id) {}

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
    PolyhedraMesh* mesh_ = nullptr;
    CellID         id_;
};

// =========================================================================
// Validation report
// =========================================================================

struct ValidationIssue {
    enum class Type {
        InvalidVertexSeed,
        FaceZeroHalfFaces,
        FaceThreeOrMoreHalfFaces,
        HalfFaceBackRefBroken,
        CellVertexCountMismatch,
        CellHalfFaceCountMismatch,
        CellHalfFaceBackRefBroken,
        NonManifoldFace,
        FaceMapInconsistency,
    };
    Type        type;
    std::string message;
};

struct ValidationReport {
    bool                          ok = true;
    std::vector<ValidationIssue> issues;
};

// =========================================================================
// Range helpers — returned by mesh.cells() / .halffaces() / .vertices()
// =========================================================================

// T = internal entity type (Vertex, HalfFace, Cell)
// Handle = public wrapper type (VertexHandle, HalfFaceHandle, CellHandle)
// ID = id type (VertexID, HalfFaceID, CellID)
template<typename T, typename Handle, typename ID>
class HandleRange {
public:
    HandleRange(PolyhedraMesh* mesh, const EntityPool<T, ID>* pool)
        : mesh_(mesh), pool_(pool) {}

    struct iterator {
        PolyhedraMesh*                              mesh;
        typename EntityPool<T, ID>::IDIterator      it;

        Handle operator*() const { return Handle(mesh, *it); }
        iterator& operator++() { ++it; return *this; }
        bool operator!=(const iterator& o) const { return it != o.it; }
    };

    iterator begin() const { return {mesh_, pool_->ids().begin()}; }
    iterator end()   const { return {mesh_, pool_->ids().end()}; }

private:
    PolyhedraMesh*              mesh_;
    const EntityPool<T, ID>*    pool_;
};

// =========================================================================
// PolyhedraMesh
// =========================================================================

class PolyhedraMesh {
public:
    PolyhedraMesh()  = default;
    ~PolyhedraMesh() = default;

    // Non-copyable (large data structure; use move or explicit clone)
    PolyhedraMesh(const PolyhedraMesh&)            = delete;
    PolyhedraMesh& operator=(const PolyhedraMesh&) = delete;
    PolyhedraMesh(PolyhedraMesh&&)                 = default;
    PolyhedraMesh& operator=(PolyhedraMesh&&)      = default;

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

    // Only succeeds if the vertex has no incident halffaces.
    bool delete_isolated_vertex(VertexHandle v);

    // ---- validity -------------------------------------------------------

    bool is_valid(VertexHandle   v)  const noexcept;
    bool is_valid(HalfFaceHandle hf) const noexcept;
    bool is_valid(CellHandle     c)  const noexcept;

    // ---- boundary queries -----------------------------------------------

    bool is_boundary(VertexHandle   v)  const;
    bool is_boundary(HalfFaceHandle hf) const;
    bool is_boundary(CellHandle     c)  const;

    // ---- element counts -------------------------------------------------

    size_t num_vertices()  const noexcept { return vertices_.size(); }
    size_t num_halffaces() const noexcept { return halffaces_.size(); }
    size_t num_cells()     const noexcept { return cells_.size(); }
    size_t num_faces()     const noexcept { return faces_.size(); }

    // ---- iteration range types (public so callers can use them with auto) --

    using VertexRangeT    = HandleRange<Vertex,   VertexHandle,   VertexID>;
    using HalfFaceRangeT  = HandleRange<HalfFace, HalfFaceHandle, HalfFaceID>;
    using CellRangeT      = HandleRange<Cell,      CellHandle,     CellID>;

    // ---- iteration ------------------------------------------------------

    VertexRangeT    vertices()  { return {this, &vertices_}; }
    HalfFaceRangeT  halffaces() { return {this, &halffaces_}; }
    CellRangeT      cells()     { return {this, &cells_}; }

    // const overloads (cast away const — handles are non-mutating in practice)
    VertexRangeT    vertices()  const { return {const_cast<PolyhedraMesh*>(this), &vertices_}; }
    HalfFaceRangeT  halffaces() const { return {const_cast<PolyhedraMesh*>(this), &halffaces_}; }
    CellRangeT      cells()     const { return {const_cast<PolyhedraMesh*>(this), &cells_}; }

    // ---- mesh-centric topology queries ----------------------------------

    std::vector<VertexHandle>   cell_vertices(CellHandle c)   const;
    std::vector<HalfFaceHandle> cell_halffaces(CellHandle c)  const;
    std::vector<CellHandle>     cell_cells(CellHandle c)      const;

    std::vector<VertexHandle>   halfface_vertices(HalfFaceHandle hf) const;
    HalfFaceHandle              halfface_opposite(HalfFaceHandle hf) const;
    CellHandle                  halfface_cell(HalfFaceHandle hf)     const;

    // ---- validation -----------------------------------------------------

    ValidationReport validate() const;

    // ---- internal access (used by handle wrappers) ----------------------
    // These are public so that the handle wrapper methods (defined below the
    // class) can call them without friendship boilerplate.

    EntityPool<Vertex,   VertexID>&   vertex_pool()   { return vertices_; }
    EntityPool<HalfFace, HalfFaceID>& halfface_pool() { return halffaces_; }
    EntityPool<Cell,     CellID>&     cell_pool()     { return cells_; }
    EntityPool<Face,     FaceID>&     face_pool()     { return faces_; }

    const EntityPool<Vertex,   VertexID>&   vertex_pool()   const { return vertices_; }
    const EntityPool<HalfFace, HalfFaceID>& halfface_pool() const { return halffaces_; }
    const EntityPool<Cell,     CellID>&     cell_pool()     const { return cells_; }
    const EntityPool<Face,     FaceID>&     face_pool()     const { return faces_; }

private:
    EntityPool<Vertex,   VertexID>   vertices_;
    EntityPool<Face,     FaceID>     faces_;
    EntityPool<HalfFace, HalfFaceID> halffaces_;
    EntityPool<Cell,     CellID>     cells_;

    std::unordered_map<FaceKey, FaceID, FaceKeyHash> face_map_;

    // Internal helpers
    void repair_vertex_seed(VertexID vid);
};

// =========================================================================
// Handle wrapper method bodies (need full PolyhedraMesh definition)
// =========================================================================

// VertexHandle
inline bool VertexHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_valid(*this);
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

// HalfFaceHandle
inline bool HalfFaceHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_valid(*this);
}
inline bool HalfFaceHandle::is_boundary() const {
    return mesh_->is_boundary(*this);
}
inline CellHandle HalfFaceHandle::cell() const {
    return mesh_->halfface_cell(*this);
}
inline HalfFaceHandle HalfFaceHandle::opposite() const {
    return mesh_->halfface_opposite(*this);
}
inline std::vector<VertexHandle> HalfFaceHandle::vertices() const {
    return mesh_->halfface_vertices(*this);
}

// CellHandle
inline bool CellHandle::is_valid() const noexcept {
    return mesh_ && mesh_->is_valid(*this);
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

} // namespace SolidMesh
