#include "polyhedra_mesh.h"
#include "edge_iterator.h"
#include <algorithm>
#include <cassert>

namespace SolidMesh {

const std::vector<FaceID> Polyhedra::EMPTY_FACE_LIST;
const std::vector<CellID> Polyhedra::EMPTY_CELL_LIST;

// ============================================================
// Vertex operations
// ============================================================

VertexID Polyhedra::add_vertex(const Vector3& pos) {
    Vertex v{};
    v.position = pos;
    v.normal   = Vector3{0.0, 0.0, 0.0};
    v.one_cell = INVALID_ID;
    return vertices_.insert(v);
}

VertexID Polyhedra::add_vertex(const Vector3& pos, const Vector3& normal) {
    Vertex v{};
    v.position = pos;
    v.normal   = normal;
    v.one_cell = INVALID_ID;
    return vertices_.insert(v);
}

void Polyhedra::remove_vertex(VertexID v) {
    assert(vertices_.is_alive(v));
    auto it = vcells_.find(v);
    assert(it == vcells_.end() || it->second.empty());
    vcells_.erase(v);
    vertices_.erase(v);
}

// ============================================================
// Cell operations
// ============================================================

CellID Polyhedra::add_cell(CellType type, const VertexID* verts, uint32_t vcount) {
    assert(vcount == vertex_count_for(type));

    Cell c{};
    c.type = type;
    c.hfaces.fill(INVALID_ID);
    c.verts.fill(INVALID_ID);
    for (uint32_t i = 0; i < vcount; ++i)
        c.verts[i] = verts[i];

    // Insert cell first to get its ID (we'll fill hfaces next)
    CellID cid = cells_.insert(c);

    const FaceDescriptor* table = face_table_for(type);
    uint32_t nf = face_count_for(type);

    for (uint32_t fi = 0; fi < nf; ++fi) {
        const FaceDescriptor& fd = table[fi];
        VertexID fverts[4];
        for (uint32_t k = 0; k < fd.vcount; ++k)
            fverts[k] = verts[fd.local_verts[k]];

        FaceKey key = make_face_key(fverts, fd.vcount);
        auto it = face_map_.find(key);

        HalfFaceID hf;
        if (it != face_map_.end()) {
            // Face already exists — this cell takes side 1
            FaceID fid = it->second;
            Face& f = faces_[fid];
            assert(f.cell[1] == INVALID_ID && "Face already has two owners");
            f.cell[1] = cid;
            hf = make_hf(fid, 1);
        } else {
            // Create new face — this cell takes side 0
            Face f{};
            f.verts.fill(INVALID_ID);
            for (uint32_t k = 0; k < fd.vcount; ++k)
                f.verts[k] = fverts[k];
            f.vert_count = static_cast<uint8_t>(fd.vcount);
            f.cell[0] = cid;
            f.cell[1] = INVALID_ID;

            FaceID fid = faces_.insert(f);
            face_map_[key] = fid;
            hf = make_hf(fid, 0);

            // Register this face for each of its edges
            for (uint32_t k = 0; k < fd.vcount; ++k) {
                VertexID a = fverts[k];
                VertexID b = fverts[(k + 1) % fd.vcount];
                edge_faces_[make_edge_key(a, b)].push_back(fid);
            }
        }

        cells_[cid].hfaces[fi] = hf;
    }

    // Update vertex adjacency
    for (uint32_t i = 0; i < vcount; ++i) {
        add_vcell(verts[i], cid);
        vertices_[verts[i]].one_cell = cid;
    }

    return cid;
}

CellID Polyhedra::add_tet(VertexID v0, VertexID v1, VertexID v2, VertexID v3) {
    VertexID vs[4] = {v0, v1, v2, v3};
    return add_cell(CellType::Tet, vs, 4);
}

CellID Polyhedra::add_hex(VertexID v0, VertexID v1, VertexID v2, VertexID v3,
                            VertexID v4, VertexID v5, VertexID v6, VertexID v7) {
    VertexID vs[8] = {v0, v1, v2, v3, v4, v5, v6, v7};
    return add_cell(CellType::Hex, vs, 8);
}

CellID Polyhedra::add_pyramid(VertexID v0, VertexID v1, VertexID v2,
                                VertexID v3, VertexID v4) {
    VertexID vs[5] = {v0, v1, v2, v3, v4};
    return add_cell(CellType::Pyramid, vs, 5);
}

CellID Polyhedra::add_prism(VertexID v0, VertexID v1, VertexID v2,
                              VertexID v3, VertexID v4, VertexID v5) {
    VertexID vs[6] = {v0, v1, v2, v3, v4, v5};
    return add_cell(CellType::Prism, vs, 6);
}

void Polyhedra::remove_cell(CellID cid) {
    assert(cells_.is_alive(cid));
    const Cell& c = cells_[cid];
    uint32_t nv = vertex_count_for(c.type);

    // Unlink from faces and remove orphaned faces
    unlink_cell_from_faces(cid);

    // Update vertex adjacency
    for (uint32_t i = 0; i < nv; ++i) {
        VertexID v = c.verts[i];
        if (v == INVALID_ID) continue;
        remove_vcell(v, cid);
        // Update one_cell if it pointed to this cell
        if (vertices_.is_alive(v) && vertices_[v].one_cell == cid) {
            auto it = vcells_.find(v);
            if (it != vcells_.end() && !it->second.empty())
                vertices_[v].one_cell = it->second.front();
            else
                vertices_[v].one_cell = INVALID_ID;
        }
    }

    cells_.erase(cid);
}

// ============================================================
// Adjacency queries
// ============================================================

const std::vector<CellID>& Polyhedra::vertex_cells(VertexID v) const {
    auto it = vcells_.find(v);
    if (it == vcells_.end()) return EMPTY_CELL_LIST;
    return it->second;
}

void Polyhedra::cell_halffaces(CellID cid, std::vector<HalfFaceID>& out) const {
    out.clear();
    const Cell& c = cells_[cid];
    uint32_t nf = face_count_for(c.type);
    for (uint32_t i = 0; i < nf; ++i)
        out.push_back(c.hfaces[i]);
}

const std::vector<FaceID>& Polyhedra::edge_faces(VertexID v0, VertexID v1) const {
    auto it = edge_faces_.find(make_edge_key(v0, v1));
    if (it == edge_faces_.end()) return EMPTY_FACE_LIST;
    return it->second;
}

void Polyhedra::edge_cells(VertexID v0, VertexID v1, std::vector<CellID>& out) const {
    out.clear();
    const auto& flist = edge_faces(v0, v1);
    if (flist.empty()) return;

    HalfFaceID seed = INVALID_ID;
    for (FaceID fid : flist) {
        const Face& f = faces_[fid];
        if (f.cell[0] != INVALID_ID) { seed = make_hf(fid, 0); break; }
        if (f.cell[1] != INVALID_ID) { seed = make_hf(fid, 1); break; }
    }
    if (seed == INVALID_ID) return;

    EdgeCirculator circ(*this, v0, v1, seed);
    HalfFaceID first = circ.halfface();
    do {
        out.push_back(circ.cell());
        ++circ;
    } while (circ.is_valid() && circ.halfface() != first);
}

// ============================================================
// ============================================================

bool Polyhedra::is_boundary_face(FaceID f) const {
    const Face& fa = faces_[f];
    return fa.cell[0] == INVALID_ID || fa.cell[1] == INVALID_ID;
}

bool Polyhedra::is_boundary_halfface(HalfFaceID hf) const {
    const Face& f = faces_[face_of(hf)];
    // A half-face is on the boundary if the opposite side has no owner
    int opp = 1 - side_of(hf);
    return f.cell[opp] == INVALID_ID;
}

bool Polyhedra::is_boundary_edge(VertexID v0, VertexID v1) const {
    const auto& flist = edge_faces(v0, v1);
    for (FaceID fid : flist)
        if (is_boundary_face(fid)) return true;
    return false;
}

bool Polyhedra::is_boundary_vertex(VertexID v) const {
    const auto& cells = vertex_cells(v);
    for (CellID cid : cells) {
        const Cell& c = cells_[cid];
        uint32_t nf = face_count_for(c.type);
        for (uint32_t i = 0; i < nf; ++i) {
            if (is_boundary_halfface(c.hfaces[i])) return true;
        }
    }
    return false;
}

// ============================================================
// Validity check
// ============================================================

bool Polyhedra::check_validity() const {
    bool ok = true;

    // Every face's cell references must be alive (or INVALID_ID for boundary)
    for (uint32_t di = 0; di < faces_.size(); ++di) {
        FaceID fid = faces_.id_of(di);
        const Face& f = faces_[fid];
        for (int s = 0; s < 2; ++s) {
            if (f.cell[s] != INVALID_ID && !cells_.is_alive(f.cell[s])) {
                ok = false;
            }
        }
        // At least one side must be owned
        if (f.cell[0] == INVALID_ID && f.cell[1] == INVALID_ID) {
            ok = false;
        }
    }

    // Every cell's half-faces must reference alive faces
    for (uint32_t di = 0; di < cells_.size(); ++di) {
        CellID cid = cells_.id_of(di);
        const Cell& c = cells_[cid];
        uint32_t nf = face_count_for(c.type);
        for (uint32_t i = 0; i < nf; ++i) {
            HalfFaceID hf = c.hfaces[i];
            if (hf == INVALID_ID) { ok = false; continue; }
            FaceID fid = face_of(hf);
            if (!faces_.is_alive(fid)) { ok = false; continue; }
            // The face must list this cell as owner on the correct side
            if (faces_[fid].cell[side_of(hf)] != cid) ok = false;
        }
    }

    // Every vertex's vcells entries must be alive cells
    for (auto& [vid, clist] : vcells_) {
        if (!vertices_.is_alive(vid)) { ok = false; continue; }
        for (CellID cid : clist)
            if (!cells_.is_alive(cid)) ok = false;
    }

    return ok;
}

// ============================================================
// Internal helpers
// ============================================================

FaceKey Polyhedra::make_face_key(const VertexID* verts, uint32_t vcount) const {
    FaceKey key;
    key.fill(INVALID_ID);
    for (uint32_t i = 0; i < vcount; ++i)
        key[i] = verts[i];
    std::sort(key.begin(), key.begin() + vcount);
    return key;
}

void Polyhedra::unlink_cell_from_faces(CellID cid) {
    const Cell& c = cells_[cid];
    uint32_t nf = face_count_for(c.type);

    for (uint32_t i = 0; i < nf; ++i) {
        HalfFaceID hf = c.hfaces[i];
        if (hf == INVALID_ID) continue;
        FaceID fid = face_of(hf);
        if (!faces_.is_alive(fid)) continue;

        Face& f = faces_[fid];
        int s = side_of(hf);
        f.cell[s] = INVALID_ID;

        remove_face_if_orphan(fid);
    }
}

void Polyhedra::remove_face_if_orphan(FaceID fid) {
    const Face& f = faces_[fid];
    if (f.cell[0] != INVALID_ID || f.cell[1] != INVALID_ID) return;

    // Remove from face_map_
    FaceKey key = make_face_key(f.verts.data(), f.vert_count);
    face_map_.erase(key);

    // Remove from edge_faces_
    for (uint32_t k = 0; k < f.vert_count; ++k) {
        VertexID a = f.verts[k];
        VertexID b = f.verts[(k + 1) % f.vert_count];
        EdgeKey ek = make_edge_key(a, b);
        auto it = edge_faces_.find(ek);
        if (it != edge_faces_.end()) {
            auto& flist = it->second;
            flist.erase(std::remove(flist.begin(), flist.end(), fid), flist.end());
            if (flist.empty()) edge_faces_.erase(it);
        }
    }

    faces_.erase(fid);
}

void Polyhedra::add_vcell(VertexID v, CellID c) {
    vcells_[v].push_back(c);
}

void Polyhedra::remove_vcell(VertexID v, CellID c) {
    auto it = vcells_.find(v);
    if (it == vcells_.end()) return;
    auto& vec = it->second;
    vec.erase(std::remove(vec.begin(), vec.end(), c), vec.end());
}

} // namespace SolidMesh
