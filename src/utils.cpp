#include "solidmesh/math/utils.h"
namespace SolidMesh
{

    double signed_volume(const SolidMesh::Vector3 &a,
                         const SolidMesh::Vector3 &b,
                         const SolidMesh::Vector3 &c,
                         const SolidMesh::Vector3 &d)
    {
        auto ab = b - a;
        auto ac = c - a;
        auto ad = d - a;
        return SolidMesh::dot(ab, SolidMesh::cross(ac, ad));
    };

    double ray_triangle_intersection(const SolidMesh::Vector3 &ray_origin,
                                     const SolidMesh::Vector3 &ray_dir,
                                     const SolidMesh::Vector3 &v0,
                                     const SolidMesh::Vector3 &v1,
                                     const SolidMesh::Vector3 &v2,
                                     double tolerance = 1e-8)
    {
        // Moller-Trumbore intersection algorithm
        auto e1 = v1 - v0;
        auto e2 = v2 - v0;
        auto pvec = SolidMesh::cross(ray_dir, e2);
        double det = SolidMesh::dot(e1, pvec);

        if (std::abs(det) < tolerance)
            return -1.0; // No intersection or ray is parallel to triangle

        double inv_det = 1.0 / det;
        auto tvec = ray_origin - v0;
        double u = SolidMesh::dot(tvec, pvec) * inv_det;
        if (u < 0.0 || u > 1.0)
            return -1.0; // Intersection outside triangle

        auto qvec = SolidMesh::cross(tvec, e1);
        double v = SolidMesh::dot(ray_dir, qvec) * inv_det;
        if (v < 0.0 || u + v > 1.0)
            return -1.0; // Intersection outside triangle

        double t = SolidMesh::dot(e2, qvec) * inv_det;
        return t >= 0.0 ? t : -1.0; // Return distance along ray or -1 if behind ray
    }

                                     
}