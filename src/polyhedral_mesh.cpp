#include "solidmesh/mesh/polyhedral_mesh.h"
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
static uint8_t compute_orientation(const std::vector<VertexID>& face_verts,
                                   const std::vector<VertexID>& hf_verts) {
    const int n = static_cast<int>(face_verts.size());
    for (int start = 0; start < n; ++start) {
        bool match = true;
        for (int i = 0; i < n; ++i)
            if (face_verts[(start + i) % n] != hf_verts[i]) { match = false; break; }
        if (match) return static_cast<uint8_t>(start);
    }
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

VertexHandle PolyhedralMesh::add_vertex(const Vector3& position) {
    Vertex v;
    v.position = position;
    VertexID id = vertices_.insert(v);
    return VertexHandle(this, id);
}

// =========================================================================
// add_cell
// =========================================================================

CellHandle PolyhedralMesh::add_cell(CellType type,
                                    const std::vector<VertexHandle>& verts) {
    const CellTopologyTraits& topo = get_cell_topology(type);

    if (static_cast<int>(verts.size()) != topo.num_vertices)
        return CellHandle();
    for (const auto& vh : verts)
        if (!is_handle_valid(vh)) return CellHandle();

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
            if (f.hf[0].has_value() && f.hf[1].has_value())
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
            // Build canonical ring from key slots + hf_verts
            Face face;
            face.vertices.resize(fd.size);
            for (int i = 0; i < fd.size; ++i)
                for (const auto& vid : hf_verts)
                    if (vid.slot == key.slots[i]) { face.vertices[i] = vid; break; }

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

    // Update vertex seeds and vertex_cell_adj_
    const Cell& built_cell = cells_.get(cid);
    for (int vi = 0; vi < static_cast<int>(vids.size()); ++vi) {
        VertexID vid = vids[vi];

        // Mesh-layer adjacency (not stored in Vertex struct)
        vertex_cell_adj_[vid.slot].push_back(cid);

        Vertex& v = vertices_.get(vid);
        if (v.one_halfface.has_value()) continue;
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

bool PolyhedralMesh::delete_cell(CellHandle ch) {
    if (!is_handle_valid(ch)) return false;
    CellID cid = ch.id();
    Cell cell  = cells_.get(cid); // copy before erasing

    // Remove from vertex_cell_adj_ (mesh-layer, not element-layer)
    for (VertexID vid : cell.vertices) {
        auto it = vertex_cell_adj_.find(vid.slot);
        if (it != vertex_cell_adj_.end()) {
            auto& vec = it->second;
            for (size_t i = 0; i < vec.size(); ++i) {
                if (vec[i] == cid) { vec[i] = vec.back(); vec.pop_back(); break; }
            }
        }
    }

    for (HalfFaceID hfid : cell.halffaces) {
        HalfFace hf = halffaces_.get(hfid); // copy
        FaceID   fid = hf.face;
        Face&    face = faces_.get(fid);

        if (face.hf[0] == hfid) {
            if (face.hf[1].has_value()) {
                face.hf[0] = face.hf[1];
                face.hf[1] = HalfFaceID{};
            } else {
                FaceKey key = make_face_key(face.vertices);
                face_map_.erase(key);
                faces_.erase(fid);
            }
        } else if (face.hf[1] == hfid) {
            face.hf[1] = HalfFaceID{};
        }

        halffaces_.erase(hfid);
    }

    cells_.erase(cid);

    for (VertexID vid : cell.vertices)
        repair_vertex_seed(vid);

    return true;
}

// =========================================================================
// delete_isolated_vertex
// =========================================================================

bool PolyhedralMesh::delete_isolated_vertex(VertexHandle vh) {
    if (!is_handle_valid(vh)) return false;
    VertexID vid = vh.id();
    auto it = vertex_cell_adj_.find(vid.slot);
    if (it != vertex_cell_adj_.end() && !it->second.empty()) return false;
    if (it != vertex_cell_adj_.end()) vertex_cell_adj_.erase(it);
    vertices_.erase(vid);
    return true;
}

// =========================================================================
// repair_vertex_seed — O(degree) via vertex_cell_adj_
// =========================================================================

void PolyhedralMesh::repair_vertex_seed(VertexID vid) {
    if (!vertices_.exist(vid)) return;
    Vertex& v = vertices_.get(vid);

    if (v.one_halfface.has_value() && halffaces_.exist(v.one_halfface))
        return;

    v.one_halfface = HalfFaceID{};

    auto it = vertex_cell_adj_.find(vid.slot);
    if (it == vertex_cell_adj_.end()) return;

    for (CellID cid : it->second) {
        if (!cells_.exist(cid)) continue;
        const Cell& cell = cells_.get(cid);
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
}

// =========================================================================
// Validity
// =========================================================================

bool PolyhedralMesh::is_handle_valid(VertexHandle v)    const noexcept {
    return v.id().has_value() && vertices_.exist(v.id());
}
bool PolyhedralMesh::is_handle_valid(FaceHandle f)      const noexcept {
    return f.id().has_value() && faces_.exist(f.id());
}
bool PolyhedralMesh::is_handle_valid(HalfFaceHandle hf) const noexcept {
    return hf.id().has_value() && halffaces_.exist(hf.id());
}
bool PolyhedralMesh::is_handle_valid(CellHandle c)      const noexcept {
    return c.id().has_value() && cells_.exist(c.id());
}

// =========================================================================
// Boundary queries
// =========================================================================

bool PolyhedralMesh::is_boundary(FaceHandle fh) const {
    if (!is_handle_valid(fh)) return false;
    return faces_.get(fh.id()).is_boundary();
}

bool PolyhedralMesh::is_boundary(HalfFaceHandle hf) const {
    if (!is_handle_valid(hf)) return false;
    return faces_.get(halffaces_.get(hf.id()).face).is_boundary();
}

bool PolyhedralMesh::is_boundary(CellHandle ch) const {
    if (!is_handle_valid(ch)) return false;
    for (HalfFaceID hfid : cells_.get(ch.id()).halffaces)
        if (faces_.get(halffaces_.get(hfid).face).is_boundary())
            return true;
    return false;
}

bool PolyhedralMesh::is_boundary(VertexHandle vh) const {
    if (!is_handle_valid(vh)) return false;
    VertexID vid = vh.id();
    auto it = vertex_cell_adj_.find(vid.slot);
    if (it == vertex_cell_adj_.end() || it->second.empty()) return false;

    for (CellID cid : it->second) {
        const Cell& cell = cells_.get(cid);
        for (HalfFaceID hfid : cell.halffaces) {
            if (!faces_.get(halffaces_.get(hfid).face).is_boundary()) continue;
            // Check if this boundary face contains the vertex
            const Face& face = faces_.get(halffaces_.get(hfid).face);
            for (VertexID fv : face.vertices)
                if (fv == vid) return true;
        }
    }
    return false;
}

// =========================================================================
// Topology queries
// =========================================================================

std::vector<VertexHandle> PolyhedralMesh::cell_vertices(CellHandle ch) const {
    std::vector<VertexHandle> result;
    if (!is_handle_valid(ch)) return result;
    for (VertexID vid : cells_.get(ch.id()).vertices)
        result.emplace_back(const_cast<PolyhedralMesh*>(this), vid);
    return result;
}

std::vector<HalfFaceHandle> PolyhedralMesh::cell_halffaces(CellHandle ch) const {
    std::vector<HalfFaceHandle> result;
    if (!is_handle_valid(ch)) return result;
    for (HalfFaceID hfid : cells_.get(ch.id()).halffaces)
        result.emplace_back(const_cast<PolyhedralMesh*>(this), hfid);
    return result;
}

std::vector<CellHandle> PolyhedralMesh::cell_cells(CellHandle ch) const {
    std::vector<CellHandle> result;
    if (!is_handle_valid(ch)) return result;
    for (HalfFaceID hfid : cells_.get(ch.id()).halffaces) {
        const Face& face = faces_.get(halffaces_.get(hfid).face);
        HalfFaceID opp = face.hf[0] == hfid ? face.hf[1] : face.hf[0];
        if (opp.has_value())
            result.emplace_back(const_cast<PolyhedralMesh*>(this), halffaces_.get(opp).cell);
    }
    return result;
}

std::vector<VertexHandle> PolyhedralMesh::halfface_vertices(HalfFaceHandle hf) const {
    std::vector<VertexHandle> result;
    if (!is_handle_valid(hf)) return result;
    const HalfFace& hfdata = halffaces_.get(hf.id());
    for (VertexID vid : oriented_vertices(faces_.get(hfdata.face), hfdata.orientation))
        result.emplace_back(const_cast<PolyhedralMesh*>(this), vid);
    return result;
}

HalfFaceHandle PolyhedralMesh::halfface_opposite(HalfFaceHandle hf) const {
    if (!is_handle_valid(hf)) return HalfFaceHandle();
    const Face& face = faces_.get(halffaces_.get(hf.id()).face);
    HalfFaceID opp = face.hf[0] == hf.id() ? face.hf[1] : face.hf[0];
    if (!opp.has_value()) return HalfFaceHandle();
    return HalfFaceHandle(const_cast<PolyhedralMesh*>(this), opp);
}

CellHandle PolyhedralMesh::halfface_cell(HalfFaceHandle hf) const {
    if (!is_handle_valid(hf)) return CellHandle();
    return CellHandle(const_cast<PolyhedralMesh*>(this), halffaces_.get(hf.id()).cell);
}

FaceHandle PolyhedralMesh::halfface_face(HalfFaceHandle hf) const {
    if (!is_handle_valid(hf)) return FaceHandle();
    return FaceHandle(const_cast<PolyhedralMesh*>(this), halffaces_.get(hf.id()).face);
}

std::vector<VertexHandle> PolyhedralMesh::face_vertices(FaceHandle fh) const {
    std::vector<VertexHandle> result;
    if (!is_handle_valid(fh)) return result;
    for (VertexID vid : faces_.get(fh.id()).vertices)
        result.emplace_back(const_cast<PolyhedralMesh*>(this), vid);
    return result;
}

std::pair<HalfFaceHandle, HalfFaceHandle>
PolyhedralMesh::face_halffaces(FaceHandle fh) const {
    if (!is_handle_valid(fh)) return {};
    const Face& f = faces_.get(fh.id());
    auto* m = const_cast<PolyhedralMesh*>(this);
    return {
        f.hf[0].has_value() ? HalfFaceHandle(m, f.hf[0]) : HalfFaceHandle(),
        f.hf[1].has_value() ? HalfFaceHandle(m, f.hf[1]) : HalfFaceHandle()
    };
}

std::vector<CellHandle> PolyhedralMesh::vertex_cells(VertexHandle vh) const {
    std::vector<CellHandle> result;
    if (!is_handle_valid(vh)) return result;
    auto it = vertex_cell_adj_.find(vh.id().slot);
    if (it == vertex_cell_adj_.end()) return result;
    result.reserve(it->second.size());
    for (CellID cid : it->second)
        result.emplace_back(const_cast<PolyhedralMesh*>(this), cid);
    return result;
}

std::vector<HalfFaceHandle> PolyhedralMesh::vertex_halffaces(VertexHandle vh) const {
    std::vector<HalfFaceHandle> result;
    if (!is_handle_valid(vh)) return result;
    VertexID vid = vh.id();
    auto it = vertex_cell_adj_.find(vid.slot);
    if (it == vertex_cell_adj_.end()) return result;

    for (CellID cid : it->second) {
        const Cell& cell = cells_.get(cid);
        const CellTopologyTraits& topo = get_cell_topology(cell.type);
        for (int fi = 0; fi < topo.num_faces; ++fi) {
            const FaceLocalDesc& fd = topo.faces[fi];
            for (int k = 0; k < fd.size; ++k) {
                if (cell.vertices[fd.vids[k]] == vid) {
                    result.emplace_back(const_cast<PolyhedralMesh*>(this), cell.halffaces[fi]);
                    break;
                }
            }
        }
    }
    return result;
}

std::vector<std::pair<VertexHandle, VertexHandle>>
PolyhedralMesh::cell_edges(CellHandle ch) const {
    std::vector<std::pair<VertexHandle, VertexHandle>> result;
    if (!is_handle_valid(ch)) return result;

    const Cell& cell = cells_.get(ch.id());
    const CellTopologyTraits& topo = get_cell_topology(cell.type);
    auto* m = const_cast<PolyhedralMesh*>(this);

    // Deduplicate edges within this cell using sorted slot pairs
    std::vector<std::pair<uint32_t, uint32_t>> seen;
    seen.reserve(12); // max edges in a hex

    for (int fi = 0; fi < topo.num_faces; ++fi) {
        const FaceLocalDesc& fd = topo.faces[fi];
        for (int k = 0; k < fd.size; ++k) {
            VertexID va = cell.vertices[fd.vids[k]];
            VertexID vb = cell.vertices[fd.vids[(k + 1) % fd.size]];
            uint32_t sa = va.slot, sb = vb.slot;
            if (sa > sb) std::swap(sa, sb);
            auto key = std::make_pair(sa, sb);
            bool found = false;
            for (const auto& e : seen) if (e == key) { found = true; break; }
            if (!found) {
                seen.push_back(key);
                result.emplace_back(VertexHandle(m, va), VertexHandle(m, vb));
            }
        }
    }
    return result;
}

std::vector<CellHandle> PolyhedralMesh::edge_cells(VertexHandle v0, VertexHandle v1) const
{
    std::vector<CellHandle> result;
    for (const auto &c : this->vertex_cells(v0))
    {
        for (const auto &v : c.vertices())
        {
            if (v == v1)
            {
                result.push_back(c);
                break;
            }
        }
    }
    return result;
}

std::vector<CellHandle> PolyhedralMesh::edge_cells_ccw(VertexHandle v0, VertexHandle v1) const
{
    // Collect all tets incident to edge (v0,v1).
    std::vector<CellHandle> incident = edge_cells(v0, v1);
    if (incident.empty()) return {};

    int N = (int)incident.size();

    // Walk the shell ring: two tets are adjacent in the shell iff they share
    // a face containing both v0 and v1 (i.e. they share a halfface whose
    // opposite halfface belongs to another shell tet).
    // We use halfface adjacency: for each shell tet, find the (up to 2)
    // halffaces whose face contains both v0 and v1; the opposite halfface's
    // cell is the shell-neighbour.

    auto contains_both = [&](CellHandle c, VertexHandle a, VertexHandle b) -> std::vector<HalfFaceHandle> {
        std::vector<HalfFaceHandle> result;
        for (auto& hf : c.halffaces()) {
            bool has_a = false, has_b = false;
            for (auto& v : hf.vertices()) {
                if (v == a) has_a = true;
                if (v == b) has_b = true;
            }
            if (has_a && has_b) result.push_back(hf);
        }
        return result;
    };

    // Build adjacency within the shell
    std::vector<std::vector<int>> adj(N);
    for (int i = 0; i < N; ++i) {
        auto shared_hfs = contains_both(incident[i], v0, v1);
        for (auto& hf : shared_hfs) {
            auto opp = hf.opposite();
            if (!opp.is_valid()) continue;
            auto nb_cell = opp.cell();
            for (int j = 0; j < N; ++j) {
                if (j != i && incident[j] == nb_cell) {
                    bool already = false;
                    for (int x : adj[i]) if (x == j) { already = true; break; }
                    if (!already) {
                        adj[i].push_back(j);
                        adj[j].push_back(i);
                    }
                    break;
                }
            }
        }
    }

    // Walk the ring starting from tet 0
    std::vector<int> order;
    order.reserve(N);
    std::vector<bool> visited(N, false);
    order.push_back(0);
    visited[0] = true;
    for (int step = 1; step < N; ++step) {
        int cur = order.back();
        bool found = false;
        for (int nb : adj[cur]) {
            if (!visited[nb]) {
                order.push_back(nb);
                visited[nb] = true;
                found = true;
                break;
            }
        }
        if (!found) break;
    }

    std::vector<CellHandle> result;
    result.reserve(order.size());
    for (int idx : order)
        result.push_back(incident[idx]);
    return result;
}


// =========================================================================
// Validation
// =========================================================================

ValidationReport PolyhedralMesh::validate() const {
    ValidationReport report;
    auto issue = [&](ValidationIssue::Type t, std::string msg) {
        report.ok = false;
        report.issues.push_back({t, std::move(msg)});
    };

    for (size_t di = 0; di < vertices_.size(); ++di) {
        VertexID vid = vertices_.id_at(di);
        const Vertex& v = vertices_.get(vid);
        if (v.one_halfface.has_value() && !halffaces_.exist(v.one_halfface)) {
            std::ostringstream ss;
            ss << "Vertex slot=" << vid.slot << " has stale one_halfface seed";
            issue(ValidationIssue::Type::InvalidVertexSeed, ss.str());
        }
    }

    for (size_t di = 0; di < faces_.size(); ++di) {
        FaceID fid = faces_.id_at(di);
        const Face& f = faces_.get(fid);
        if (!f.hf[0].has_value() && !f.hf[1].has_value()) {
            std::ostringstream ss;
            ss << "Face slot=" << fid.slot << " has zero halffaces";
            issue(ValidationIssue::Type::FaceZeroHalfFaces, ss.str());
        }
        if (f.hf[0].has_value() && !halffaces_.exist(f.hf[0])) {
            std::ostringstream ss;
            ss << "Face slot=" << fid.slot << " hf[0] is stale";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
        if (f.hf[1].has_value() && !halffaces_.exist(f.hf[1])) {
            std::ostringstream ss;
            ss << "Face slot=" << fid.slot << " hf[1] is stale";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
    }

    for (size_t di = 0; di < halffaces_.size(); ++di) {
        HalfFaceID hfid = halffaces_.id_at(di);
        const HalfFace& hf = halffaces_.get(hfid);

        if (!faces_.exist(hf.face)) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " has stale face ref";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
            continue;
        }
        if (!cells_.exist(hf.cell)) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " has stale cell ref";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
            continue;
        }
        const Cell& cell = cells_.get(hf.cell);
        if (hf.local_face_index >= cell.halffaces.size() ||
            cell.halffaces[hf.local_face_index] != hfid) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " back-ref to cell broken";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
        const Face& face = faces_.get(hf.face);
        if (face.hf[0] != hfid && face.hf[1] != hfid) {
            std::ostringstream ss;
            ss << "HalfFace slot=" << hfid.slot << " not listed in its face";
            issue(ValidationIssue::Type::HalfFaceBackRefBroken, ss.str());
        }
    }

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
            if (!halffaces_.exist(hfid)) {
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

    for (const auto& [key, fid] : face_map_)
        if (!faces_.exist(fid))
            issue(ValidationIssue::Type::FaceMapInconsistency, "face_map_ contains stale FaceID");

    for (const auto& [slot, cids] : vertex_cell_adj_) {
        for (CellID cid : cids) {
            if (!cells_.exist(cid)) {
                std::ostringstream ss;
                ss << "vertex_cell_adj_ slot=" << slot << " contains stale CellID";
                issue(ValidationIssue::Type::VertexAdjInconsistency, ss.str());
            }
        }
    }

    return report;
}

} // namespace SolidMesh
