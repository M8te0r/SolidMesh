#pragma once

#include "PolyhedraMesh.h"
#include <vector>

namespace SolidMesh {

// EdgeCirculator: traverses all cells sharing edge (v0, v1) in ring order.
//
// Algorithm (rotate-around-edge):
//   Given current_hf (a half-face of cell C containing edge v0-v1):
//   1. Within C, find the OTHER half-face that also contains v0-v1.
//   2. Cross to the opposite half-face of that face.
//   3. Repeat until we return to start (closed ring) or hit a boundary.
//
// Usage:
//   EdgeCirculator circ(mesh, v0, v1);
//   if (circ.is_valid()) {
//       do {
//           CellID c = circ.cell();
//           ++circ;
//       } while (circ.is_valid() && !circ.at_start());
//   }
class EdgeCirculator {
public:
    EdgeCirculator(const Polyhedra& mesh, VertexID v0, VertexID v1);
    EdgeCirculator(const Polyhedra& mesh, VertexID v0, VertexID v1, HalfFaceID seed);

    bool is_valid()    const { return current_hf_ != INVALID_ID; }
    bool at_start()    const { return current_hf_ == start_hf_; }
    bool is_boundary() const { return boundary_; }

    CellID     cell()     const;
    HalfFaceID halfface() const { return current_hf_; }

    EdgeCirculator& operator++();

private:
    const Polyhedra* mesh_;
    VertexID v0_, v1_;
    HalfFaceID start_hf_;
    HalfFaceID current_hf_;
    bool boundary_;  // true if the ring is open (edge is on boundary)

    // Within the cell owning hf, find the other half-face containing v0_ and v1_.
    HalfFaceID next_hf_in_cell(HalfFaceID hf) const;

    // Check if a face contains both v0_ and v1_.
    bool face_has_edge(FaceID fid) const;
};

} // namespace SolidMesh

// ---- Implementation (header-only) ----

namespace SolidMesh {

// Private constructor used by edge_cells above
inline EdgeCirculator::EdgeCirculator(const Polyhedra& mesh,
                                       VertexID v0, VertexID v1,
                                       HalfFaceID seed)
    : mesh_(&mesh), v0_(v0), v1_(v1),
      start_hf_(seed), current_hf_(seed), boundary_(false)
{}

inline EdgeCirculator::EdgeCirculator(const Polyhedra& mesh,
                                       VertexID v0, VertexID v1)
    : mesh_(&mesh), v0_(v0), v1_(v1),
      start_hf_(INVALID_ID), current_hf_(INVALID_ID), boundary_(false)
{
    const auto& flist = mesh.edge_faces(v0, v1);
    if (flist.empty()) return;
    const Face& f = mesh.face(flist[0]);
    if (f.cell[0] != INVALID_ID)      start_hf_ = make_hf(flist[0], 0);
    else if (f.cell[1] != INVALID_ID) start_hf_ = make_hf(flist[0], 1);
    current_hf_ = start_hf_;
}

inline CellID EdgeCirculator::cell() const {
    if (current_hf_ == INVALID_ID) return INVALID_ID;
    FaceID fid = face_of(current_hf_);
    return mesh_->face(fid).cell[side_of(current_hf_)];
}

inline bool EdgeCirculator::face_has_edge(FaceID fid) const {
    const Face& f = mesh_->face(fid);
    bool has_v0 = false, has_v1 = false;
    for (uint32_t i = 0; i < f.vert_count; ++i) {
        if (f.verts[i] == v0_) has_v0 = true;
        if (f.verts[i] == v1_) has_v1 = true;
    }
    return has_v0 && has_v1;
}

inline HalfFaceID EdgeCirculator::next_hf_in_cell(HalfFaceID hf) const {
    CellID cid = mesh_->face(face_of(hf)).cell[side_of(hf)];
    if (cid == INVALID_ID) return INVALID_ID;
    const Cell& c = mesh_->cell(cid);
    uint32_t nf = face_count_for(c.type);
    for (uint32_t i = 0; i < nf; ++i) {
        HalfFaceID candidate = c.hfaces[i];
        if (candidate == hf) continue;
        if (face_has_edge(face_of(candidate))) return candidate;
    }
    return INVALID_ID;
}

inline EdgeCirculator& EdgeCirculator::operator++() {
    if (current_hf_ == INVALID_ID) return *this;

    HalfFaceID next_in_cell = next_hf_in_cell(current_hf_);
    if (next_in_cell == INVALID_ID) {
        current_hf_ = INVALID_ID;
        boundary_ = true;
        return *this;
    }

    HalfFaceID crossed = opposite_hf(next_in_cell);
    // Check if the crossed half-face has an owner
    FaceID fid = face_of(crossed);
    if (!mesh_->faces_map().is_alive(fid) ||
        mesh_->face(fid).cell[side_of(crossed)] == INVALID_ID) {
        current_hf_ = INVALID_ID;
        boundary_ = true;
        return *this;
    }

    current_hf_ = crossed;
    return *this;
}

} // namespace SolidMesh
