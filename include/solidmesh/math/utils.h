#pragma once
#include "solidmesh/math/vector3.h"

namespace SolidMesh
{
    double signed_volume(const SolidMesh::Vector3 &a,
                         const SolidMesh::Vector3 &b,
                         const SolidMesh::Vector3 &c,
                         const SolidMesh::Vector3 &d);

    double ray_triangle_intersection(const SolidMesh::Vector3 &ray_origin,
                                     const SolidMesh::Vector3 &ray_dir,
                                     const SolidMesh::Vector3 &v0,
                                     const SolidMesh::Vector3 &v1,
                                     const SolidMesh::Vector3 &v2,
                                     double tolerance = 1e-8);

}