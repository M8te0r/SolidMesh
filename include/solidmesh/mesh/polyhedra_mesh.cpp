#include "solidmesh/mesh/polyhedra_mesh.h"
#include <sstream>
#include <cassert>

namespace SolidMesh {

// Compute the oriented vertex ring of a halfface from its face's canonical ring.
// orientation & 0x0F = start offset; orientation & 0x10 = flip.
static std::vector<VertexID> oriented_vertices(const Face& face, uint8_t orientation) {
    const auto& cv = face.vertices;
    const int n = static_cast<int>(cv.size());
    const int start = orientation & 0x0F;
    const bool flip = (orientation & 0x10) != 0;
    std::vector<VertexID> result(n);
    for (int i = 0; i < n; ++i) {
        int idx = flip ? (start - i + n) % n : (start + i) % n;
        result[i] = cv[idx];
    }
    return result;
}

// Compute the orientation encoding that maps face.vertices -> hf_verts.
// Returns 0xFF if no valid encoding found (should never happen for valid input).
static uint8_t compute_orientation(const std::vector<VertexID>& face_verts,
                                   const std::vector<VertexID>& hf_verts) {
    const int n = static_cast<int>(face_verts.size());
    // Try forward rotations
    for (int start = 0; start < n; ++start) {
        bool match = true;
        for (int i = 0; i < n; ++i)
            if (face_verts[(start + i) % n] != hf_verts[i]) { match = false; break; }
        if (match) return static_cast<uint8_t>(start);
    }
    // Try flipped rotations (flip flag = 0x10)
    for (int start = 0; start < n; ++start) {
        bool match = true;
        for (int i = 0; i < n; ++i)
            if (face_verts[(start - i + n) % n] != hf_verts[i]) { match = false; break; }
        if (match) return static_cast<uint8_t>(0x10 | start);
    }
    return 0xFF; // unreachable for valid topology
}

// =========================================================================
// add_vertex
// =========================================================================

VertexHandle PolyhedraMesh::add_vertex(const Vector3& position) {
    Vertex v;
    v.position = position;
    VertexID id = vertices_.insert(v);
    return VertexHandle(this, id);
}

// =========================================================================
// add_cell
// =========================================================================

CellHandle PolyhedraMesh::add_cell(CellType type,
                                    const std::vector<VertexHandle>& verts) {
    const CellTopologyTraits& topo = get_cell_topology(type);

    if (static_cast<int>(verts.size()) != topo.num_vertices)
        return CellHandle();
    for (const auto& vh : verts)
        if (!is_valid(vh)) return CellHandle();

    std::vector<VertexID> vids;
    vids.reserve(verts.size());
    for (const auto& vh : verts)
        vids.push_back(vh.id());

    // Pre-check: would any local face become non-manifold?
    for (int fi = 0; fi < topo.num_faces; ++fi) {
        const FaceLocalDesc& fd = topo.faces[fi];
        std::vector<VertexID> hf_verts;
        for (int k = 0; k < fd.size; ++k)
            hf_verts.push_back(vids[fd.vids[k]]);
        FaceKey key = make_face_key(hf_verts);
        auto it = face_map_.find(key);
        if (it != face_map_.end()) {
            const Face& f = faces_.get(it->second);
            if (f.hf[0].is_valid() && f.hf[1].is_valid())
                return CellHandle(); // would create non-manifold face
        }
    }

    // Create the Cell
    Cell cell;
    cell.type     = type;
    cell.vertices = vids;
    cell.halffaces.resize(topo.num_faces);
    CellID cid = cells_.insert(cell);

    // Create halffaces and wire up faces
    for (int fi = 0; fi < topo.num_faces; ++fi) {
        const FaceLocalDesc& fd = topo.faces[fi];

        std::vector<VertexID> hf_verts;
        hf_verts.reserve(fd.size);
        for (int k = 0; k < fd.size; ++k)
            hf_verts.push_back(vids[fd.vids[k]]);

        FaceKey key = make_face_key(hf_verts);

        HalfFace hf;
        hf.cell             = cid;
        hf.local_face_index = static_cast<uint8_t>(fi);

        auto it = face_map_.find(key);
        if (it == face_map_.end()) {
            Face face;
            face.vertices = key.vertices; // canonical ring
            FaceID fid = faces_.insert(face);
            face_map_[key] = fid;

            hf.face        = fid;
            hf.orientation = compute_orientation(faces_.get(fid).vertices, hf_verts);
            HalfFaceID hfid = halffaces_.insert(hf);
            faces_.get(fid).hf[0] = hfid;
            cells_.get(cid).halffaces[fi] = hfid;
        } else {
            FaceID fid = it->second;
            hf.face        = fid;
            hf.orientation = compute_orientation(faces_.get(fid).vertices, hf_verts);
            HalfFaceID hfid = halffaces_.insert(hf);
            faces_.get(fid).hf[1] = hfid;
            cells_.get(cid).halffaces[fi] = hfid;
        }
    }

    // Update vertex seeds — use local cell data, no global scan needed
    const Cell& built_cell = cells_.get(cid);
    for (int vi = 0; vi < static_cast<int>(vids.size()); ++vi) {
        Vertex& v = vertices_.get(vids[vi]);
        if (v.one_halfface.is_valid()) continue;
        // Find any halfface of this cell that contains vids[vi]
        for (int fi = 0; fi < topo.num_faces; ++fi) {
            const FaceLocalDesc& fd = topo.faces[fi];
            for (int k = 0; k < fd.size; ++k) {
                if (fd.vids[k] == vi) {
                    v.one_halfface = built_cell.halffaces[fi];
                    goto next_vert;
                }
            }
        }
        next_vert:;
    }

    return CellHandle(this, cid);
}

// =========================================================================
// delete_cell
// =========================================================================

bool PolyhedraMesh::delete_cell(CellHandle ch) {
    if (!is_valid(ch)) return false;
    CellID cid = ch.id();
    Cell cell  = cells_.get(cid); // copy before erasing

    for (HalfFaceID hfid : cell.halffaces) {
        HalfFace hf = halffaces_.get(hfid); // copy
        FaceID   fid = hf.face;
        Face&    face = faces_.get(fid);

        if (face.hf[0] == hfid) {
            if (face.hf[1].is_valid()) {
                // Interior face becomes boundary
                face.hf[0] = face.hf[1];
                face.hf[1] = HalfFaceID{};
            } else {
                // Was already boundary — remove face entirely
                FaceKey key = make_face_key(face.vertices);
                face_map_.erase(key);
                faces_.erase(fid);
            }
        } else if (face.hf[1] == hfid) {
            face.hf[1] = HalfFaceID{};
            // face becomes boundary; hf[0] stays
        }

        halffaces_.erase(hfid);
    }

    cells_.erase(cid);

    // Repair vertex seeds that pointed into deleted halffaces
    for (VertexID vid : cell.vertices)
        repair_vertex_seed(vid);

    return true;
}

// =========================================================================
// delete_isolated_vertex
// =========================================================================

bool PolyhedraMesh::delete_isolated_vertex(VertexHandle vh) {
    if (!is_valid(vh)) return false;
    const Vertex& v = vertices_.get(vh.id());
    if (v.one_halfface.is_valid()) return false; // not isolated
    vertices_.erase(vh.id());
    return true;
}

// =========================================================================
// repair_vertex_seed
// =========================================================================

void PolyhedraMesh::repair_vertex_seed(VertexID vid) {
    if (!vertices_.alive(vid)) return;
    Vertex& v = vertices_.get(vid);

    // If current seed is still alive, nothing to do
    if (v.one_halfface.is_valid() && halffaces_.alive(v.one_halfface))
        return;

    v.one_halfface = HalfFaceID{}; // reset

    // Search remaining cells for any halfface containing this vertex.
    // Uses cell topology tables to avoid iterating halfface vertices.
    for (size_t di = 0; di < cells_.size(); ++di) {
        const Cell& cell = cells_.dense()[di];
        const CellTopologyTraits& topo = get_cell_topology(cell.type);
        for (int vi = 0; vi < static_cast<int>(cell.vertices.size()); ++vi) {
            if (cell.vertices[vi] != vid) continue;
            for (int fi = 0; fi < topo.num_faces; ++fi) {
                const FaceLocalDesc& fd = topo.faces[fi];
                for (int k = 0; k < fd.size; ++k) {
                    if (fd.vids[k] == vi) {
                        v.one_halfface = cell.halffaces[fi];
                        return;
                    }
                }
            }
        }
    }
    // No incident halfface found — vertex is now isolated
}

// =========================================================================
// Validity
// =========================================================================

bool PolyhedraMesh::is_valid(VertexHandle v) const noexcept {
    return v.id().is_valid() && vertices_.alive(v.id());
}
bool PolyhedraMesh::is_valid(HalfFaceHandle hf) const noexcept {
    return hf.id().is_valid() && halffaces_.alive(hf.id());
}
bool PolyhedraMesh::is_valid(CellHandle c) const noexcept {
    return c.id().is_valid() && cells_.alive(c.id());
}

// =========================================================================
// Boundary queries
// =========================================================================

bool PolyhedraMesh::is_boundary(HalfFaceHandle hf) const {
    if (!is_valid(hf)) return false;
    FaceID fid = halffaces_.get(hf.id()).face;
    return faces_.get(fid).is_boundary();
}

bool PolyhedraMesh::is_boundary(CellHandle ch) const {
    if (!is_valid(ch)) return false;
    for (HalfFaceID hfid : cells_.get(ch.id()).halffaces)
        if (faces_.get(halffaces_.get(hfid).face).is_boundary())
            return true;
    return false;
}

bool PolyhedraMesh::is_boundary(VertexHandle vh) const {
    if (!is_valid(vh)) return false;
    const Vertex& v = vertices_.get(vh.id());
    if (!v.one_halfface.is_valid()) return false; // isolated vertex

    // Walk all cells containing this vertex via the cell pool,
    // checking each of their halffaces for boundary status.
    // This avoids the O(N) halfface scan by using cell.vertices membership.
    VertexID vid = vh.id();
    for (size_t di = 0; di < cells_.size(); ++di) {
        const Cell& cell = cells_.dense()[di];
        bool contains = false;
        for (VertexID cv : cell.vertices)
            if (cv == vid) { contains = true; break; }
        if (!contains) continue;
        for (HalfFaceID hfid : cell.halffaces)
            if (faces_.get(halffaces_.get(hfid).face).is_boundary())
                return true;
    }
    return false;
}

// =========================================================================
// Topology queries
// =========================================================================

std::vector<VertexHandle> PolyhedraMesh::cell_vertices(CellHandle ch) const {
    std::vector<VertexHandle> result;
    if (!is_valid(ch)) return result;
    for (VertexID vid : cells_.get(ch.id()).vertices)
        result.emplace_back(const_cast<PolyhedraMesh*>(this), vid);
    return result;
}

std::vector<HalfFaceHandle> PolyhedraMesh::cell_halffaces(CellHandle ch) const {
    std::vector<HalfFaceHandle> result;
    if (!is_valid(ch)) return result;
    for (HalfFaceID hfid : cells_.get(ch.id()).halffaces)
        result.emplace_back(const_cast<PolyhedraMesh*>(this), hfid);
    return result;
}

std::vector<CellHandle> PolyhedraMesh::cell_cells(CellHandle ch) const {
    std::vector<CellHandle> result;
    if (!is_valid(ch)) return result;
    for (HalfFaceID hfid : cells_.get(ch.id()).halffaces) {
        const Face& face = faces_.get(halffaces_.get(hfid).face);
        // Find the opposite halfface
        HalfFaceID opp = face.hf[0] == hfid ? face.hf[1] : face.hf[0];
        if (opp.is_valid()) {
            CellID ncid = halffaces_.get(opp).cell;
            result.emplace_back(const_cast<PolyhedraMesh*>(this), ncid);
        }
    }
    return result;
}

std::vector<VertexHandle> PolyhedraMesh::halfface_vertices(HalfFaceHandle hf) const {
    std::vector<VertexHandle> result;
    if (!is_valid(hf)) return result;
    const HalfFace& hfdata = halffaces_.get(hf.id());
    const Face& face = faces_.get(hfdata.face);
    for (VertexID vid : oriented_vertices(face, hfdata.orientation))
        result.emplace_back(const_cast<PolyhedraMesh*>(this), vid);
    return result;
}

HalfFaceHandle PolyhedraMesh::halfface_opposite(HalfFaceHandle hf) const {
    if (!is_valid(hf)) return HalfFaceHandle();
    const Face& face = faces_.get(halffaces_.get(hf.id()).face);
    HalfFaceID opp = face.hf[0] == hf.id() ? face.hf[1] : face.hf[0];
    if (!opp.is_valid()) return HalfFaceHandle();
    return HalfFaceHandle(const_cast<PolyhedraMesh*>(this), opp);
}

CellHandle PolyhedraMesh::halfface_cell(HalfFaceHandle hf) const {
    if (!is_valid(hf)) return CellHandle();
    return CellHandle(const_cast<PolyhedraMesh*>(this),
                      halffaces_.get(hf.id()).cell);
}

// =========================================================================
// Validation
// =========================================================================

ValidationReport PolyhedraMesh::validate() const {
    ValidationReport report;
    auto issue = [&](ValidationIssue::Type t, std::string msg) {
        report.ok = false;
        report.issues.push_back({t, std::move(msg)});
    };

    // Check vertices
    for (size_t di = 0; di < vertices_.size(); ++di) {
        VertexID vid = vertices_.id_at(di);
        const Vertex& v = vertices_.get(vid);
        if (v.one_halfface.is_valid() && !halffaces_.alive(v.one_halfface)) {
            std::ostringstream ss;
            ss << "Vertex slot=" << vid.slot << " has stale one_halfface seed";
            issue(ValidationIssue::Type::InvalidVertexSeed, ss.str());
        }
    }

    // Check faces
    for (size_t di = 0; di < faces_.size(); ++di) {
        FaceID fid = faces_.id_at(di);
        const Face& f = faces_.get(fid);
        int count = (f.hf[0].is_valid() ? 1 : 0) + (f.hf[1].is_valid() ? 1 : 0);
        if (count == 0) {
            std::ostringstream ss;
            ss << "Face slot=" << fid.slot << " has zero halffaces";
            issue(ValidationIssue::Type::FaceZeroHalfFaces, ss.str());
        }
        if (f.hf[0].is_valid() && !halffaces_.alive(f.hf[0])) {
            std::ostringstream ss;
            ss << "Face slot=" << fid.slot << " hf[0] is stale";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
        if (f.hf[1].is_valid() && !halffaces_.alive(f.hf[1])) {
            std::ostringstream ss;
            ss << "Face slot=" << fid.slot << " hf[1] is stale";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
    }

    // Check halffaces
    for (size_t di = 0; di < halffaces_.size(); ++di) {
        HalfFaceID hfid = halffaces_.id_at(di);
        const HalfFace& hf = halffaces_.get(hfid);

        if (!faces_.alive(hf.face)) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " has stale face ref";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
            continue;
        }
        if (!cells_.alive(hf.cell)) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " has stale cell ref";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
            continue;
        }

        // Back-reference: cell must list this halfface
        const Cell& cell = cells_.get(hf.cell);
        if (hf.local_face_index >= cell.halffaces.size() ||
            cell.halffaces[hf.local_face_index] != hfid) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " back-ref to cell broken";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }

        // Face must list this halfface
        const Face& face = faces_.get(hf.face);
        if (face.hf[0] != hfid && face.hf[1] != hfid) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " not listed in its face";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
    }

    // Check cells
    for (size_t di = 0; di < cells_.size(); ++di) {
        CellID cid = cells_.id_at(di);
        const Cell& cell = cells_.get(cid);
        const CellTopologyTraits& topo = get_cell_topology(cell.type);

        if (static_cast<int>(cell.vertices.size()) != topo.num_vertices) {
            std::ostringstream ss;
            ss << "Cell slot=" << cid.slot << " vertex count mismatch";
            issue(ValidationIssue::Type::CellVertexCountMismatch, ss.str());
        }
        if (static_cast<int>(cell.halffaces.size()) != topo.num_faces) {
            std::ostringstream ss;
            ss << "Cell slot=" << cid.slot << " halfface count mismatch";
            issue(ValidationIssue::Type::CellHalfFaceCountMismatch, ss.str());
        }
        for (int fi = 0; fi < static_cast<int>(cell.halffaces.size()); ++fi) {
            HalfFaceID hfid = cell.halffaces[fi];
            if (!halffaces_.alive(hfid)) {
                std::ostringstream ss;
                ss << "Cell slot=" << cid.slot << " halfface[" << fi << "] is stale";
                issue(ValidationIssue::Type::CellHalfFaceBackRefBroken, ss.str());
                continue;
            }
            const HalfFace& hf = halffaces_.get(hfid);
            if (hf.cell != cid || hf.local_face_index != fi) {
                std::ostringstream ss;
                ss << "Cell slot=" << cid.slot << " halfface[" << fi << "] back-ref broken";
                issue(ValidationIssue::Type::CellHalfFaceBackRefBroken, ss.str());
            }
        }
    }

    // Check face_map_ consistency
    for (const auto& [key, fid] : face_map_) {
        if (!faces_.alive(fid)) {
            issue(ValidationIssue::Type::FaceMapInconsistency,
                  "face_map_ contains stale FaceID");
        }
    }

    return report;
}

} // namespace SolidMesh
