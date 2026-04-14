#include "TopologyOperations.h"
#include "mesh/EdgeIterator.h"
#include <algorithm>
#include <cassert>

namespace SolidMesh {

// ============================================================
// Geometry helpers
// ============================================================

double tet_signed_volume(const Polyhedra& mesh,
                         VertexID v0, VertexID v1, VertexID v2, VertexID v3) {
    const Vector3& p0 = mesh.vertex(v0).position;
    const Vector3& p1 = mesh.vertex(v1).position;
    const Vector3& p2 = mesh.vertex(v2).position;
    const Vector3& p3 = mesh.vertex(v3).position;
    Vector3 a = p1 - p0;
    Vector3 b = p2 - p0;
    Vector3 c = p3 - p0;
    return dot(a, cross(b, c)) / 6.0;
}

bool tet_is_valid(const Polyhedra& mesh,
                  VertexID v0, VertexID v1, VertexID v2, VertexID v3) {
    return tet_signed_volume(mesh, v0, v1, v2, v3) > 0.0;
}

// ============================================================
// swap23
// ============================================================

bool swap23(Polyhedra& mesh, HalfFaceID hf,
            std::array<CellID, 3>& new_cells_out) {
    FaceID shared_fid = face_of(hf);
    const Face& shared_face = mesh.face(shared_fid);

    // Precondition: both sides must be tets
    CellID cid0 = shared_face.cell[0];
    CellID cid1 = shared_face.cell[1];
    if (cid0 == INVALID_ID || cid1 == INVALID_ID) return false;
    if (mesh.cell(cid0).type != CellType::Tet) return false;
    if (mesh.cell(cid1).type != CellType::Tet) return false;
    if (shared_face.vert_count != 3) return false;

    // Shared face vertices {a, b, c}
    VertexID a = shared_face.verts[0];
    VertexID b = shared_face.verts[1];
    VertexID c = shared_face.verts[2];

    // Find apex of each tet (the vertex not on the shared face)
    auto find_apex = [&](CellID cid) -> VertexID {
        const Cell& cell = mesh.cell(cid);
        for (uint32_t i = 0; i < 4; ++i) {
            VertexID v = cell.verts[i];
            if (v != a && v != b && v != c) return v;
        }
        return INVALID_ID;
    };

    VertexID d = find_apex(cid0);  // apex of cell owning side 0
    VertexID e = find_apex(cid1);  // apex of cell owning side 1
    if (d == INVALID_ID || e == INVALID_ID) return false;

    // Validate: all 3 new tets must have positive volume
    // New tets: {a,b,d,e}, {b,c,d,e}, {c,a,d,e}
    if (!tet_is_valid(mesh, a, b, d, e)) return false;
    if (!tet_is_valid(mesh, b, c, d, e)) return false;
    if (!tet_is_valid(mesh, c, a, d, e)) return false;

    // Remove old cells
    mesh.remove_cell(cid0);
    mesh.remove_cell(cid1);

    // Add 3 new tets
    new_cells_out[0] = mesh.add_tet(a, b, d, e);
    new_cells_out[1] = mesh.add_tet(b, c, d, e);
    new_cells_out[2] = mesh.add_tet(c, a, d, e);

    return true;
}

// ============================================================
// swap32
// ============================================================

bool swap32(Polyhedra& mesh, VertexID v0, VertexID v1,
            std::array<CellID, 2>& new_cells_out) {
    std::vector<CellID> ring;
    mesh.edge_cells(v0, v1, ring);
    if (ring.size() != 3) return false;

    // All 3 must be tets
    for (CellID cid : ring)
        if (mesh.cell(cid).type != CellType::Tet) return false;

    // Collect the 3 "outer" vertices (not v0 or v1) from each tet
    VertexID outer[3];
    for (int i = 0; i < 3; ++i) {
        const Cell& c = mesh.cell(ring[i]);
        outer[i] = INVALID_ID;
        for (uint32_t k = 0; k < 4; ++k) {
            VertexID v = c.verts[k];
            if (v != v0 && v != v1) {
                // Pick the one not already found
                bool dup = false;
                for (int j = 0; j < i; ++j) if (outer[j] == v) { dup = true; break; }
                if (!dup) { outer[i] = v; break; }
            }
        }
        if (outer[i] == INVALID_ID) return false;
    }

    VertexID p = outer[0], q = outer[1], r = outer[2];

    // New tets: {v0, p, q, r} and {v1, p, q, r}
    if (!tet_is_valid(mesh, v0, p, q, r)) return false;
    if (!tet_is_valid(mesh, v1, p, q, r)) return false;

    for (CellID cid : ring) mesh.remove_cell(cid);

    new_cells_out[0] = mesh.add_tet(v0, p, q, r);
    new_cells_out[1] = mesh.add_tet(v1, p, q, r);
    return true;
}

// ============================================================
// swap22
// ============================================================

bool swap22(Polyhedra& mesh, HalfFaceID hf,
            std::array<CellID, 2>& new_cells_out) {
    FaceID shared_fid = face_of(hf);
    const Face& shared_face = mesh.face(shared_fid);

    CellID cid0 = shared_face.cell[0];
    CellID cid1 = shared_face.cell[1];
    if (cid0 == INVALID_ID || cid1 == INVALID_ID) return false;
    if (mesh.cell(cid0).type != CellType::Tet) return false;
    if (mesh.cell(cid1).type != CellType::Tet) return false;
    if (shared_face.vert_count != 3) return false;

    VertexID a = shared_face.verts[0];
    VertexID b = shared_face.verts[1];
    VertexID c = shared_face.verts[2];

    auto find_apex = [&](CellID cid) -> VertexID {
        const Cell& cell = mesh.cell(cid);
        for (uint32_t i = 0; i < 4; ++i) {
            VertexID v = cell.verts[i];
            if (v != a && v != b && v != c) return v;
        }
        return INVALID_ID;
    };

    VertexID d = find_apex(cid0);
    VertexID e = find_apex(cid1);
    if (d == INVALID_ID || e == INVALID_ID) return false;

    // Flip: replace shared face {a,b,c} with new face {a,d,e} or similar.
    // The two new tets share edge (d,e) instead of face (a,b,c).
    // New tets: {a, b, d, e} and {a, c, e, d}  — one valid flip configuration.
    // Check all permutations for positive volume.
    struct Config { VertexID w0,w1,w2,w3, x0,x1,x2,x3; };
    Config configs[] = {
        {a,b,d,e, a,c,e,d},
        {a,c,d,e, a,b,e,d},
        {b,c,d,e, a,b,c,d},  // degenerate — same as swap23 partial
    };
    for (auto& cfg : configs) {
        if (tet_is_valid(mesh, cfg.w0,cfg.w1,cfg.w2,cfg.w3) &&
            tet_is_valid(mesh, cfg.x0,cfg.x1,cfg.x2,cfg.x3)) {
            mesh.remove_cell(cid0);
            mesh.remove_cell(cid1);
            new_cells_out[0] = mesh.add_tet(cfg.w0,cfg.w1,cfg.w2,cfg.w3);
            new_cells_out[1] = mesh.add_tet(cfg.x0,cfg.x1,cfg.x2,cfg.x3);
            return true;
        }
    }
    return false;
}

// ============================================================
// face_split
// ============================================================

bool face_split(Polyhedra& mesh, FaceID fid,
                VertexID& new_vertex_out,
                std::vector<CellID>& new_cells_out) {
    new_cells_out.clear();
    const Face& f = mesh.face(fid);

    // Compute centroid
    Vector3 centroid{0.0, 0.0, 0.0};
    for (uint32_t i = 0; i < f.vert_count; ++i)
        centroid += mesh.vertex(f.verts[i]).position;
    centroid = centroid * (1.0 / f.vert_count);

    new_vertex_out = mesh.add_vertex(centroid);
    VertexID nv = new_vertex_out;

    // For each cell owning this face, rebuild it with the split
    for (int s = 0; s < 2; ++s) {
        CellID cid = f.cell[s];
        if (cid == INVALID_ID) continue;
        const Cell& c = mesh.cell(cid);
        if (c.type != CellType::Tet) continue;  // only tet supported for now

        // Find the apex (vertex not on the face)
        VertexID apex = INVALID_ID;
        for (uint32_t i = 0; i < 4; ++i) {
            VertexID v = c.verts[i];
            bool on_face = false;
            for (uint32_t k = 0; k < f.vert_count; ++k)
                if (f.verts[k] == v) { on_face = true; break; }
            if (!on_face) { apex = v; break; }
        }
        if (apex == INVALID_ID) continue;

        // Split: for each edge of the face, create a new tet with nv and apex
        for (uint32_t k = 0; k < f.vert_count; ++k) {
            VertexID fa = f.verts[k];
            VertexID fb = f.verts[(k + 1) % f.vert_count];
            new_cells_out.push_back(mesh.add_tet(fa, fb, nv, apex));
        }
    }

    // Remove original cells (collect first to avoid iterator invalidation)
    std::vector<CellID> to_remove;
    for (int s = 0; s < 2; ++s)
        if (f.cell[s] != INVALID_ID) to_remove.push_back(f.cell[s]);
    for (CellID cid : to_remove)
        if (mesh.cells_map().is_alive(cid)) mesh.remove_cell(cid);

    return !new_cells_out.empty();
}

// ============================================================
// edge_split
// ============================================================

bool edge_split(Polyhedra& mesh, VertexID v0, VertexID v1,
                VertexID& new_vertex_out,
                std::vector<CellID>& new_cells_out) {
    new_cells_out.clear();

    std::vector<CellID> ring;
    mesh.edge_cells(v0, v1, ring);
    if (ring.empty()) return false;

    // Midpoint
    Vector3 mid = (mesh.vertex(v0).position + mesh.vertex(v1).position) * 0.5;
    new_vertex_out = mesh.add_vertex(mid);
    VertexID nv = new_vertex_out;

    // For each tet in the ring, split it into 2 tets
    std::vector<CellID> to_remove = ring;
    for (CellID cid : ring) {
        const Cell& c = mesh.cell(cid);
        if (c.type != CellType::Tet) continue;

        // Find the 2 vertices not on the edge
        VertexID others[2];
        uint32_t oi = 0;
        for (uint32_t i = 0; i < 4 && oi < 2; ++i) {
            VertexID v = c.verts[i];
            if (v != v0 && v != v1) others[oi++] = v;
        }
        if (oi != 2) continue;

        // Two new tets: replace v0-v1 edge with v0-nv and nv-v1
        new_cells_out.push_back(mesh.add_tet(v0, nv, others[0], others[1]));
        new_cells_out.push_back(mesh.add_tet(nv, v1, others[0], others[1]));
    }

    for (CellID cid : to_remove)
        if (mesh.cells_map().is_alive(cid)) mesh.remove_cell(cid);

    return !new_cells_out.empty();
}

// ============================================================
// edge_collapse
// ============================================================

bool edge_collapse(Polyhedra& mesh, VertexID v0, VertexID v1,
                   const Vector3& new_position,
                   std::vector<CellID>& removed_cells_out) {
    removed_cells_out.clear();

    // Link condition check: the intersection of the 1-rings of v0 and v1
    // must equal the set of vertices on cells containing both v0 and v1.
    // (Simplified: just check that no non-degenerate cell would invert.)
    const auto& cells0 = mesh.vertex_cells(v0);
    const auto& cells1 = mesh.vertex_cells(v1);

    // Cells containing both v0 and v1 are degenerate after collapse
    std::vector<CellID> degenerate;
    for (CellID cid : cells0) {
        const Cell& c = mesh.cell(cid);
        uint32_t nv = vertex_count_for(c.type);
        for (uint32_t i = 0; i < nv; ++i) {
            if (c.verts[i] == v1) { degenerate.push_back(cid); break; }
        }
    }

    // Cells containing v1 but not v0 will have v1 replaced by v0
    std::vector<CellID> to_update;
    for (CellID cid : cells1) {
        bool is_deg = false;
        for (CellID d : degenerate) if (d == cid) { is_deg = true; break; }
        if (!is_deg) to_update.push_back(cid);
    }

    // Validate: after moving v0 to new_position, no cell in to_update inverts
    // (Only check tets for now)
    Vector3 old_pos = mesh.vertex(v0).position;
    mesh.vertex(v0).position = new_position;
    bool valid = true;
    for (CellID cid : to_update) {
        const Cell& c = mesh.cell(cid);
        if (c.type != CellType::Tet) continue;
        VertexID vs[4];
        uint32_t vi = 0;
        for (uint32_t i = 0; i < 4; ++i)
            vs[vi++] = (c.verts[i] == v1) ? v0 : c.verts[i];
        if (!tet_is_valid(mesh, vs[0], vs[1], vs[2], vs[3])) {
            valid = false; break;
        }
    }
    mesh.vertex(v0).position = old_pos;
    if (!valid) return false;

    // Perform collapse: remove degenerate cells, rebuild to_update cells
    for (CellID cid : degenerate) {
        removed_cells_out.push_back(cid);
        mesh.remove_cell(cid);
    }

    // Rebuild cells that referenced v1 with v0 instead
    for (CellID cid : to_update) {
        const Cell& c = mesh.cell(cid);
        CellType type = c.type;
        uint32_t nv = vertex_count_for(type);
        VertexID new_verts[8];
        for (uint32_t i = 0; i < nv; ++i)
            new_verts[i] = (c.verts[i] == v1) ? v0 : c.verts[i];
        mesh.remove_cell(cid);
        mesh.add_cell(type, new_verts, nv);
    }

    // Move v0 to new position, remove v1
    mesh.vertex(v0).position = new_position;
    mesh.remove_vertex(v1);

    return true;
}

// ============================================================
// edge_suppress
// ============================================================

bool edge_suppress(Polyhedra& mesh, VertexID v, FaceID& merged_face_out) {
    merged_face_out = INVALID_ID;

    // v must have exactly 2 incident edges (valence 2 in the mesh graph)
    // Collect all unique neighbors of v
    const auto& cells = mesh.vertex_cells(v);
    std::vector<VertexID> neighbors;
    for (CellID cid : cells) {
        const Cell& c = mesh.cell(cid);
        uint32_t nv = vertex_count_for(c.type);
        for (uint32_t i = 0; i < nv; ++i) {
            VertexID nb = c.verts[i];
            if (nb == v) continue;
            bool found = false;
            for (VertexID x : neighbors) if (x == nb) { found = true; break; }
            if (!found) neighbors.push_back(nb);
        }
    }
    if (neighbors.size() != 2) return false;

    // Collapse v onto the midpoint of its two neighbors
    Vector3 mid = (mesh.vertex(neighbors[0]).position +
                  mesh.vertex(neighbors[1]).position) * 0.5;
    std::vector<CellID> removed;
    return edge_collapse(mesh, v, neighbors[0], mid, removed);
}

} // namespace SolidMesh
