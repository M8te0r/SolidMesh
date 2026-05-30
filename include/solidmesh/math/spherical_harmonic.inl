#include "spherical_harmonic.h"
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
} // namespace SolidMesh
