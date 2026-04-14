#pragma once

#include "mesh/PolyhedraMesh.h"
#include <array>
#include <vector>

namespace SolidMesh {

// ============================================================
// Atomic topological operations for H-Morph
//
// All operations return true on success, false if preconditions fail.
// On failure the mesh is left unmodified.
// ============================================================

// swap23: Replace 2 tets sharing a face with 3 tets sharing an edge.
//
// Precondition: hf and opposite_hf(hf) are both tet half-faces.
// The shared face has vertices {a, b, c}.
// The two tets have apices d (cell owning hf) and e (cell owning opposite).
// Result: 3 new tets sharing edge (d,e): {a,b,d,e}, {b,c,d,e}, {c,a,d,e}.
// The 2 original tets are removed.
bool swap23(Polyhedra& mesh, HalfFaceID hf,
            std::array<CellID, 3>& new_cells_out);

// swap32: Replace 3 tets sharing an edge with 2 tets sharing a face.
// (Inverse of swap23.)
//
// Precondition: edge (v0,v1) is shared by exactly 3 tets.
// Result: 2 new tets. The 3 original tets are removed.
bool swap32(Polyhedra& mesh, VertexID v0, VertexID v1,
            std::array<CellID, 2>& new_cells_out);

// swap22: Flip the shared face between two tets (2D diagonal flip in 3D).
//
// Precondition: hf and opposite_hf(hf) are both tet half-faces,
//               and the shared face is a triangle.
// This is a degenerate case — in pure tet meshes swap22 is only valid
// when the two tets form a convex hull. Returns false otherwise.
bool swap22(Polyhedra& mesh, HalfFaceID hf,
            std::array<CellID, 2>& new_cells_out);

// face_split: Insert a vertex at the centroid of a face, splitting it.
//
// A triangular face shared by 2 tets is split into 3 triangles,
// producing 6 new tets (3 per original tet).
// A quad face shared by 2 cells is split into 4 quads.
// Returns the new vertex ID and the IDs of all new cells.
bool face_split(Polyhedra& mesh, FaceID f,
                VertexID& new_vertex_out,
                std::vector<CellID>& new_cells_out);

// edge_split: Insert a vertex at the midpoint of an edge.
//
// All cells around the edge are split. Each tet becomes 2 tets.
// Returns the new vertex ID and the IDs of all new cells.
bool edge_split(Polyhedra& mesh, VertexID v0, VertexID v1,
                VertexID& new_vertex_out,
                std::vector<CellID>& new_cells_out);

// edge_collapse: Merge two vertices connected by an edge.
//
// v1 is merged into v0 (v0 moves to new_position).
// Degenerate cells (those containing both v0 and v1) are removed.
// Returns false if the collapse would create non-manifold topology
// (link condition check).
bool edge_collapse(Polyhedra& mesh, VertexID v0, VertexID v1,
                   const Vector3& new_position,
                   std::vector<CellID>& removed_cells_out);

// edge_suppress: Remove a valence-2 interior vertex.
//
// Removes vertex v that has exactly 2 incident edges in the mesh graph.
// Merges the two edges into one. Used to clean up after edge_split reversals.
bool edge_suppress(Polyhedra& mesh, VertexID v,
                   FaceID& merged_face_out);

// ---- Geometry helpers used by topological ops ----

// Signed volume of a tet (positive = correct orientation).
double tet_signed_volume(const Polyhedra& mesh,
                         VertexID v0, VertexID v1, VertexID v2, VertexID v3);

// Returns true if the tet has positive (non-degenerate) volume.
bool tet_is_valid(const Polyhedra& mesh,
                  VertexID v0, VertexID v1, VertexID v2, VertexID v3);

} // namespace SolidMesh
