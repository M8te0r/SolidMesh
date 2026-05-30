#include "quaternion.h"
namespace SolidMesh
{
    inline constexpr double SphericalHarmonic::clamp(double val, double low, double high)
    {
        if (val < low)
            return low;
        if (val > high)
            return high;
        return val;
    }

    inline void SphericalHarmonic::to_vector(double phi, double theta, double *dir)
    {
        const double r = std::sin(theta);
        dir[0] = r * std::cos(phi);
        dir[1] = r * std::sin(phi);
        dir[2] = std::cos(theta);
    }

    inline void SphericalHarmonic::to_spherical_coords(const double *unit_dir, double *phi, double *theta)
    {
        *theta = std::acos(clamp(unit_dir[2], -1.0, 1.0));
        *phi = std::atan2(unit_dir[1], unit_dir[0]);
    }

    inline constexpr double SphericalHarmonic::sh00(const double *) { return 0.28209479177387814; }

    inline constexpr double SphericalHarmonic::sh1n1(const double *d) { return -0.48860251190291992 * d[1]; }
    inline constexpr double SphericalHarmonic::sh10(const double *d) { return 0.48860251190291992 * d[2]; }
    inline constexpr double SphericalHarmonic::sh1p1(const double *d) { return -0.48860251190291992 * d[0]; }

    inline constexpr double SphericalHarmonic::sh2n2(const double *d) { return 1.0925484305920792 * d[0] * d[1]; }
    inline constexpr double SphericalHarmonic::sh2n1(const double *d) { return -1.0925484305920792 * d[1] * d[2]; }
    inline constexpr double SphericalHarmonic::sh20(const double *d) { return 0.31539156525252005 * (-d[0] * d[0] - d[1] * d[1] + 2.0 * d[2] * d[2]); }
    inline constexpr double SphericalHarmonic::sh2p1(const double *d) { return -1.0925484305920792 * d[0] * d[2]; }
    inline constexpr double SphericalHarmonic::sh2p2(const double *d) { return 0.54627421529603959 * (d[0] * d[0] - d[1] * d[1]); }

    inline constexpr double SphericalHarmonic::sh3n3(const double *d) { return -0.59004358992664352 * d[1] * (3.0 * d[0] * d[0] - d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh3n2(const double *d) { return 2.8906114426405538 * d[0] * d[1] * d[2]; }
    inline constexpr double SphericalHarmonic::sh3n1(const double *d) { return -0.45704579946446577 * d[1] * (4.0 * d[2] * d[2] - d[0] * d[0] - d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh30(const double *d) { return 0.3731763325901154 * d[2] * (2.0 * d[2] * d[2] - 3.0 * d[0] * d[0] - 3.0 * d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh3p1(const double *d) { return -0.45704579946446577 * d[0] * (4.0 * d[2] * d[2] - d[0] * d[0] - d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh3p2(const double *d) { return 1.4453057213202769 * d[2] * (d[0] * d[0] - d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh3p3(const double *d) { return -0.59004358992664352 * d[0] * (d[0] * d[0] - 3.0 * d[1] * d[1]); }

    inline constexpr double SphericalHarmonic::sh4n4(const double *d) { return 2.5033429417967046 * d[0] * d[1] * (d[0] * d[0] - d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh4n3(const double *d) { return -1.7701307697799304 * d[1] * d[2] * (3.0 * d[0] * d[0] - d[1] * d[1]); }
    inline constexpr double SphericalHarmonic::sh4n2(const double *d) { return 0.94617469575756008 * d[0] * d[1] * (7.0 * d[2] * d[2] - 1.0); }
    inline constexpr double SphericalHarmonic::sh4n1(const double *d) { return -0.66904654355728921 * d[1] * d[2] * (7.0 * d[2] * d[2] - 3.0); }

    inline constexpr double SphericalHarmonic::sh40(const double *d)
    {
        const double z2 = d[2] * d[2];
        return 0.10578554691520431 * (35.0 * z2 * z2 - 30.0 * z2 + 3.0);
    }

    inline constexpr double SphericalHarmonic::sh4p1(const double *d) { return -0.66904654355728921 * d[0] * d[2] * (7.0 * d[2] * d[2] - 3.0); }
    inline constexpr double SphericalHarmonic::sh4p2(const double *d) { return 0.47308734787878004 * (d[0] * d[0] - d[1] * d[1]) * (7.0 * d[2] * d[2] - 1.0); }
    inline constexpr double SphericalHarmonic::sh4p3(const double *d) { return -1.7701307697799304 * d[0] * d[2] * (d[0] * d[0] - 3.0 * d[1] * d[1]); }

    inline constexpr double SphericalHarmonic::sh4p4(const double *d)
    {
        const double x2 = d[0] * d[0];
        const double y2 = d[1] * d[1];
        return 0.62583573544917614 * (x2 * (x2 - 3.0 * y2) - y2 * (3.0 * x2 - y2));
    }



    inline double SphericalHarmonic::eval_sh(int l, int m, double phi, double theta)
    {
        double dir[3];
        to_vector(phi, theta, dir);
        return eval_sh(l, m, dir);
    }

    inline constexpr double SphericalHarmonic::eval_sh(int l, int m, const double *unit_dir)
    {
        switch (l)
        {
        case 0:
            return sh00(unit_dir);
        case 1:
            switch (m)
            {
            case -1:
                return sh1n1(unit_dir);
            case 0:
                return sh10(unit_dir);
            case 1:
                return sh1p1(unit_dir);
            }
            break;
        case 2:
            switch (m)
            {
            case -2:
                return sh2n2(unit_dir);
            case -1:
                return sh2n1(unit_dir);
            case 0:
                return sh20(unit_dir);
            case 1:
                return sh2p1(unit_dir);
            case 2:
                return sh2p2(unit_dir);
            }
            break;
        case 3:
            switch (m)
            {
            case -3:
                return sh3n3(unit_dir);
            case -2:
                return sh3n2(unit_dir);
            case -1:
                return sh3n1(unit_dir);
            case 0:
                return sh30(unit_dir);
            case 1:
                return sh3p1(unit_dir);
            case 2:
                return sh3p2(unit_dir);
            case 3:
                return sh3p3(unit_dir);
            }
            break;
        case 4:
            switch (m)
            {
            case -4:
                return sh4n4(unit_dir);
            case -3:
                return sh4n3(unit_dir);
            case -2:
                return sh4n2(unit_dir);
            case -1:
                return sh4n1(unit_dir);
            case 0:
                return sh40(unit_dir);
            case 1:
                return sh4p1(unit_dir);
            case 2:
                return sh4p2(unit_dir);
            case 3:
                return sh4p3(unit_dir);
            case 4:
                return sh4p4(unit_dir);
            }
            break;
        }

        return 0.0;
    }

    inline double SphericalHarmonic::eval_sum(int order, const double *coeffs, double phi, double theta)
    {
        double dir[3];
        to_vector(phi, theta, dir);
        return eval_sum(order, coeffs, dir);
    }

    inline constexpr double SphericalHarmonic::eval_sum(int order, const double *coeffs, const double *unit_dir)
    {
        double sum = 0.0;
        const int max_l = order < max_order ? order : max_order;
        for (int l = 0; l <= max_l; l++)
        {
            for (int m = -l; m <= l; m++)
            {
                sum += eval_sh(l, m, unit_dir) * coeffs[index(l, m)];
            }
        }
        return sum;
    }

    inline double spherical_harmonic_rotation_delta(int i, int j)
    {
        return i == j ? 1.0 : 0.0;
    }

    inline double spherical_harmonic_rotation_get(const double *wigner_d, int coeff_count, int l, int m, int n)
    {
        return wigner_d[SphericalHarmonic::index(l, m) + SphericalHarmonic::index(l, n) * coeff_count];
    }

    inline void spherical_harmonic_rotation_set(double *wigner_d, int coeff_count, int l, int m, int n, double value)
    {
        wigner_d[SphericalHarmonic::index(l, m) + SphericalHarmonic::index(l, n) * coeff_count] = value;
    }

    inline double spherical_harmonic_rotation_p(int i, int a, int b, int l, const double *wigner_d, int coeff_count)
    {
        if (b == l)
        {
            return spherical_harmonic_rotation_get(wigner_d, coeff_count, 1, i, 1) *
                       spherical_harmonic_rotation_get(wigner_d, coeff_count, l - 1, a, l - 1) -
                   spherical_harmonic_rotation_get(wigner_d, coeff_count, 1, i, -1) *
                       spherical_harmonic_rotation_get(wigner_d, coeff_count, l - 1, a, -l + 1);
        }
        if (b == -l)
        {
            return spherical_harmonic_rotation_get(wigner_d, coeff_count, 1, i, 1) *
                       spherical_harmonic_rotation_get(wigner_d, coeff_count, l - 1, a, -l + 1) +
                   spherical_harmonic_rotation_get(wigner_d, coeff_count, 1, i, -1) *
                       spherical_harmonic_rotation_get(wigner_d, coeff_count, l - 1, a, l - 1);
        }
        return spherical_harmonic_rotation_get(wigner_d, coeff_count, 1, i, 0) *
               spherical_harmonic_rotation_get(wigner_d, coeff_count, l - 1, a, b);
    }

    inline double spherical_harmonic_rotation_u(int m, int n, int l, const double *wigner_d, int coeff_count)
    {
        return spherical_harmonic_rotation_p(0, m, n, l, wigner_d, coeff_count);
    }

    inline double spherical_harmonic_rotation_v(int m, int n, int l, const double *wigner_d, int coeff_count)
    {
        if (m == 0)
        {
            return spherical_harmonic_rotation_p(1, 1, n, l, wigner_d, coeff_count) +
                   spherical_harmonic_rotation_p(-1, -1, n, l, wigner_d, coeff_count);
        }
        if (m > 0)
        {
            return spherical_harmonic_rotation_p(1, m - 1, n, l, wigner_d, coeff_count) *
                       std::sqrt(1.0 + spherical_harmonic_rotation_delta(m, 1)) -
                   spherical_harmonic_rotation_p(-1, -m + 1, n, l, wigner_d, coeff_count) *
                       (1.0 - spherical_harmonic_rotation_delta(m, 1));
        }
        return spherical_harmonic_rotation_p(1, m + 1, n, l, wigner_d, coeff_count) *
                   (1.0 - spherical_harmonic_rotation_delta(m, -1)) +
               spherical_harmonic_rotation_p(-1, -m - 1, n, l, wigner_d, coeff_count) *
                   std::sqrt(1.0 + spherical_harmonic_rotation_delta(m, -1));
    }

    inline double spherical_harmonic_rotation_w(int m, int n, int l, const double *wigner_d, int coeff_count)
    {
        if (m == 0)
        {
            return 0.0;
        }
        if (m > 0)
        {
            return spherical_harmonic_rotation_p(1, m + 1, n, l, wigner_d, coeff_count) +
                   spherical_harmonic_rotation_p(-1, -m - 1, n, l, wigner_d, coeff_count);
        }
        return spherical_harmonic_rotation_p(1, m - 1, n, l, wigner_d, coeff_count) -
               spherical_harmonic_rotation_p(-1, -m + 1, n, l, wigner_d, coeff_count);
    }

    inline void spherical_harmonic_rotation_uvws(int m, int n, int l, double *u, double *v, double *w)
    {
        const double d = spherical_harmonic_rotation_delta(m, 0);
        const double denom = std::abs(n) == l ? 2.0 * l * (2.0 * l - 1.0) : (l + n) * (l - n);
        const int abs_m = std::abs(m);

        *u = std::sqrt((l + m) * (l - m) / denom);
        *v = 0.5 * std::sqrt((1.0 + d) * (l + abs_m - 1.0) * (l + abs_m) / denom) * (1.0 - 2.0 * d);
        *w = -0.5 * std::sqrt((l - abs_m - 1.0) * (l - abs_m) / denom) * (1.0 - d);
    }

    inline void SphericalHarmonic::create_sp_rotation(int order, const double* quaternion, double *wigner_d)
    {
        const Quaternion q{quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
        const int coeff_count = coefficient_count(order);
        for (int i = 0; i < coeff_count * coeff_count; i++)
        {
            wigner_d[i] = 0.0;
        }

        spherical_harmonic_rotation_set(wigner_d, coeff_count, 0, 0, 0, 1.0);
        if (order == 0)
        {
            return;
        }

        double rotation_mat[9];
        q.to_matrix(rotation_mat);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, -1, -1, rotation_mat[1 + 3 * 1]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, -1, 0, -rotation_mat[1 + 3 * 2]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, -1, 1, rotation_mat[1 + 3 * 0]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, 0, -1, -rotation_mat[2 + 3 * 1]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, 0, 0, rotation_mat[2 + 3 * 2]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, 0, 1, -rotation_mat[2 + 3 * 0]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, 1, -1, rotation_mat[0 + 3 * 1]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, 1, 0, -rotation_mat[0 + 3 * 2]);
        spherical_harmonic_rotation_set(wigner_d, coeff_count, 1, 1, 1, rotation_mat[0 + 3 * 0]);

        for (int l = 2; l <= order; l++)
        {
            for (int m = -l; m <= l; m++)
            {
                for (int n = -l; n <= l; n++)
                {
                    double u, v, w;
                    spherical_harmonic_rotation_uvws(m, n, l, &u, &v, &w);

                    double value = 0.0;
                    if (u != 0.0)
                    {
                        value += u * spherical_harmonic_rotation_u(m, n, l, wigner_d, coeff_count);
                    }
                    if (v != 0.0)
                    {
                        value += v * spherical_harmonic_rotation_v(m, n, l, wigner_d, coeff_count);
                    }
                    if (w != 0.0)
                    {
                        value += w * spherical_harmonic_rotation_w(m, n, l, wigner_d, coeff_count);
                    }

                    spherical_harmonic_rotation_set(wigner_d, coeff_count, l, m, n, value);
                }
            }
        }
    }

    inline void SphericalHarmonic::sp_rotate(int order, const double *coeffs, const double *wigner_d, double *result)
    {
        const int coeff_count = coefficient_count(order);
        double *rotated = result;
        if (result == coeffs)
        {
            rotated = new double[coeff_count];
        }

        for (int l = 0; l <= order; l++)
        {
            for (int m = -l; m <= l; m++)
            {
                double value = 0.0;
                for (int n = -l; n <= l; n++)
                {
                    value += spherical_harmonic_rotation_get(wigner_d, coeff_count, l, m, n) * coeffs[index(l, n)];
                }
                rotated[index(l, m)] = value;
            }
        }

        if (rotated != result)
        {
            for (int i = 0; i < coeff_count; i++)
            {
                result[i] = rotated[i];
            }
            delete[] rotated;
        }
    }
} // namespace SolidMesh
