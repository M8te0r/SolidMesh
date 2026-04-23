#pragma once
#include "solidmesh/math/vector3.h"

namespace SolidMesh
{
    double signed_volume(const Vector3 &a,
                         const Vector3 &b,
                         const Vector3 &c,
                         const Vector3 &d);

    bool is_point_inside_triangle(const Vector3 &p,
                              const Vector3 &a,
                              const Vector3 &b,
                              const Vector3 &c);

    /// Determine whether two line segments A,B intersect
    ///
    /// A: p + t*r :  t \in [0,1]
    /// B: q + u*s :  u \in [0,1]
    ///
    /// @param[in] p  3-vector origin of segment A
    /// @param[in] r  3-vector direction of segment A
    /// @param[in] q  3-vector origin of segment B
    /// @param[in] s  3-vector direction of segment B
    /// @param[out] t  scalar point of intersection along segment A, t \in [0,1]
    /// @param[out] u  scalar point of intersection along segment B, u \in [0,1]
    /// @param[in] eps precision
    /// @return true if intersection
    bool intersect_line_line(const Vector3 &p, const Vector3 &r,
                                  const Vector3 &q, const Vector3 &s,
                                  double *out_t, double *out_u,
                                  double tolerance = 1e-8);

    /**
     * @brief Check if a ray intersects with a triangle facet
     * 
     * @param ray_origin 
     * @param ray_dir 
     * @param v0 triangle vertex 0, counter-clockwise order is assumed
     * @param v1 triangle vertex 1, counter-clockwise order is assumed
     * @param v2 triangle vertex 2, counter-clockwise order is assumed
     * @param tolerance 
     * @return double 
     */
    double intersect_ray_triangle(const Vector3 &ray_origin,
                                     const Vector3 &ray_dir,
                                     const Vector3 &v0,
                                     const Vector3 &v1,
                                     const Vector3 &v2,
                                     double tolerance = 1e-8);

    /**
     * @brief Check if a line segment intersects with a triangle edges when they are coplanar
     * 
     * @param p segment start
     * @param q segment end
     * @param v0 triangle vertex 0, counter-clockwise order is assumed
     * @param v1 triangle vertex 1, counter-clockwise order is assumed
     * @param v2 triangle vertex 2, counter-clockwise order is assumed
     * @param is_edge_intersect output array of size 3, is_edge_intersect[i] is set to true if pq intersects with edge (v_i, v_(i+1)%3)
     * @param intersect_edge_params output array of size 3, intersect_params[i] is set to the parameter value of the intersection point on edge (v_i, v_(i+1)%3)
     * @param tolerance 
     * @return true 
     * @return false 
     */
    bool intersect_line_triangle_coplanar(const Vector3 &p,
                                             const Vector3 &q,
                                             const Vector3 &v0,
                                             const Vector3 &v1,
                                             const Vector3 &v2,
                                             std::array<bool,3>* is_edge_intersect,
                                             std::array<double,3>* intersect_edge_params,
                                             double tolerance = 1e-8);

}