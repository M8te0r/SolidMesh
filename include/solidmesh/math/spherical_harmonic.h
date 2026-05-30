#pragma once

#include <cmath>

namespace SolidMesh
{

    // Real spherical harmonics, using the same phi/theta convention as the
    // Google reference implementation:
    // phi   - rotation around the Z axis, in [0, 2pi]
    // theta - angle down from the Z axis, in [0, pi]

    /**
     * @brief Real spherical harmonics, Google reference implementation
     * l - level, [0,4], isoline of zero zone 
     * m - mode, [-4,4], 
     * each spherical function cooud represented by linear combination of spherical harmonic basis Y_l^m
     */
    struct SphericalHarmonic
    {
        static constexpr int max_order = 4;
        // 从l=0到l=order总共需要的球谐系数数量（每阶共有 2 * l + 1 个系数）
        static constexpr int coefficient_count(int order) { return (order + 1) * (order + 1); }
        // 把二维球谐索引(l,m)映射到一维数组下标
        static constexpr int index(int l, int m) { return l * (l + 1) + m; }

        static constexpr double clamp(double val, double low, double high);

        static void to_vector(double phi, double theta, double *dir);
        static void to_spherical_coords(const double *unit_dir, double *phi, double *theta);

        // spherical harmonic basis
        // l=0
        static constexpr double sh00(const double *d);
        // l=1
        static constexpr double sh1n1(const double *d);
        static constexpr double sh10(const double *d);
        static constexpr double sh1p1(const double *d);
        // l=2
        static constexpr double sh2n2(const double *d);
        static constexpr double sh2n1(const double *d);
        static constexpr double sh20(const double *d);
        static constexpr double sh2p1(const double *d);
        static constexpr double sh2p2(const double *d);
        // l=3
        static constexpr double sh3n3(const double *d);
        static constexpr double sh3n2(const double *d);
        static constexpr double sh3n1(const double *d);
        static constexpr double sh30(const double *d);
        static constexpr double sh3p1(const double *d);
        static constexpr double sh3p2(const double *d);
        static constexpr double sh3p3(const double *d);
        // l=4
        static constexpr double sh4n4(const double *d);
        static constexpr double sh4n3(const double *d);
        static constexpr double sh4n2(const double *d);
        static constexpr double sh4n1(const double *d);
        static constexpr double sh40(const double *d);
        static constexpr double sh4p1(const double *d);
        static constexpr double sh4p2(const double *d);
        static constexpr double sh4p3(const double *d);
        static constexpr double sh4p4(const double *d);


        // 计算某个球谐基函数Y_l^m在某个球坐标方向上的值
        static double eval_sh(int l, int m, double phi, double theta);
        static constexpr double eval_sh(int l, int m, const double *unit_dir);

        // 用一组球谐系数重建某个球坐标方向上的函数值
        static double eval_sum(int order, const double *coeffs, double phi, double theta);
        static constexpr double eval_sum(int order, const double *coeffs, const double *unit_dir);


        // 根据quaternion(w,x,y,z)，为l阶sp basis系数(=1、3、5、...)创造一个col-major旋转矩阵wigner_d
        static void create_sp_band_rotation(int l, const double *quaternion, double *wigner_d);
        // 根据quaternion(w,x,y,z)，为order阶sph展开的所有系数(=1+3+5+7+...)创造一个col-major旋转矩阵wigner_d
        static void create_sp_rotation(int order, const double* quaternion, double *mat);
        // 给定band=l的sp basis的系数coeffs，和对应的旋转矩阵wigner_d，得到系数旋转后的结果result
        static void sp_band_rotate(int l, const double *coeffs, const double *wigner_d, double *result);
        // 给定一个order阶sp展开的全部系数，和对应的旋转矩阵mat，得到系数旋转后的结果result
        static void sp_rotate(int order, const double *coeffs, const double *mat, double *result);
    };

} // namespace SolidMesh

#include "spherical_harmonic.inl"
