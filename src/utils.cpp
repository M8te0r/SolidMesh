#include "solidmesh/math/utils.h"
namespace SolidMesh
{

    double signed_volume(const Vector3 &a,
                         const Vector3 &b,
                         const Vector3 &c,
                         const Vector3 &d)
    {
        auto ab = b - a;
        auto ac = c - a;
        auto ad = d - a;
        return dot(ab, cross(ac, ad));
    };

    bool is_point_inside_triangle(const Vector3 &p, const Vector3 &a, const Vector3 &b, const Vector3 &c)
    {
        Vector3 v0 = c - a;
        Vector3 v1 = b - a;
        Vector3 v2 = p - a;

        double dot00 = dot(v0, v0);
        double dot01 = dot(v0, v1);
        double dot02 = dot(v0, v2);
        double dot11 = dot(v1, v1);
        double dot12 = dot(v1, v2);

        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    bool intersect_line_line(const Vector3 &p, const Vector3 &r,
                             const Vector3 &q, const Vector3 &s,
                             double *a_t, double *a_u,
                             double eps = 1e-8)
    {
        // https://github.com/libigl/libigl/blob/main/include/igl/segment_segment_intersect.cpp

        // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
        // Search intersection between two segments
        // p + t*r :  t \in [0,1]
        // q + u*s :  u \in [0,1]

        // p + t * r = q + u * s  // x s
        // t(r x s) = (q - p) x s
        // t = (q - p) x s / (r x s)

        // (r x s) ~ 0 --> directions are parallel, they will never cross
        auto rxs = cross(r, s);
        if (rxs.norm() <= eps)
            return false;

        int sign;

        double u;
        // u = (q − p) × r / (r × s)
        auto u1 = cross(q - p, r);
        sign = dot(u1, rxs) > 0 ? 1 : -1;
        u = u1.norm() / rxs.norm();
        u = u * sign;

        double t;
        // t = (q - p) x s / (r x s)
        auto t1 = cross(q - p, s);
        sign = dot(t1, rxs) > 0 ? 1 : -1;
        t = t1.norm() / rxs.norm();
        t = t * sign;

        *a_t = t;
        *a_u = u;

        if ((u - 1.) > eps || u < -eps)
            return false;

        if ((t - 1.) > eps || t < -eps)
            return false;

        return true;
    }

    double intersect_ray_triangle(const Vector3 &ray_origin,
                                  const Vector3 &ray_dir,
                                  const Vector3 &v0,
                                  const Vector3 &v1,
                                  const Vector3 &v2,
                                  double tolerance = 1e-8)
    {
        // Moller-Trumbore intersection algorithm
        auto e1 = v1 - v0;
        auto e2 = v2 - v0;
        auto pvec = cross(ray_dir, e2);
        double det = dot(e1, pvec);

        if (std::abs(det) < tolerance)
            return -1.0; // No intersection or ray is parallel to triangle

        double inv_det = 1.0 / det;
        auto tvec = ray_origin - v0;
        double u = dot(tvec, pvec) * inv_det;
        if (u < 0.0 || u > 1.0)
            return -1.0; // Intersection outside triangle

        auto qvec = cross(tvec, e1);
        double v = dot(ray_dir, qvec) * inv_det;
        if (v < 0.0 || u + v > 1.0)
            return -1.0; // Intersection outside triangle

        double t = dot(e2, qvec) * inv_det;
        return t >= 0.0 ? t : -1.0; // Return distance along ray or -1 if behind ray
    }

    bool intersect_line_triangle_coplanar(const Vector3 &p,
                                          const Vector3 &q,
                                          const Vector3 &v0,
                                          const Vector3 &v1,
                                          const Vector3 &v2,
                                          std::array<bool, 3> *is_edge_intersect,
                                          std::array<double, 3> *intersect_edge_params,
                                          double tolerance)
    {
        is_edge_intersect->fill(false);
        intersect_edge_params->fill(std::numeric_limits<double>::quiet_NaN());

        // pq均在内部，未与三角形发生相交
        if (is_point_inside_triangle(p, v0, v1, v2) && is_point_inside_triangle(q, v0, v1, v2))
            return false;

        //
        Vector3 pq_dir = (q - p).normalize();

        Vector3 e0_dir = (v1 - v0).normalize();
        Vector3 e1_dir = (v2 - v1).normalize();
        Vector3 e2_dir = (v0 - v2).normalize();

        std::array<std::pair<Vector3, Vector3>, 3> edges = {
            std::make_pair(v0, e0_dir),
            std::make_pair(v1, e1_dir),
            std::make_pair(v2, e2_dir)};

        for (int i = 0; i < 3; ++i)
        {
            double t, u;
            if (intersect_line_line(edges[i].first, edges[i].second, p, pq_dir, &t, &u, tolerance))
            {
                (*is_edge_intersect)[i] = true;
                (*intersect_edge_params)[i] = t;
            }
            else
            {
                (*is_edge_intersect)[i] = false;
            }
        }

        return (*is_edge_intersect)[0] || (*is_edge_intersect)[1] || (*is_edge_intersect)[2];
    }

}