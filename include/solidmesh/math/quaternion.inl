#include "vector3.h"

namespace SolidMesh
{
        inline Quaternion Quaternion::fromRotationMatrix(const double* mat)
        {
            const double trace = mat[0] + mat[4] + mat[8];
            if (trace > 0.)
            {
                const double s = 2. * ::std::sqrt(trace + 1.);
                return Quaternion{0.25 * s, (mat[5] - mat[7]) / s, (mat[6] - mat[2]) / s, (mat[1] - mat[3]) / s};
            }
            if (mat[0] > mat[4] && mat[0] > mat[8])
            {
                const double s = 2. * ::std::sqrt(1. + mat[0] - mat[4] - mat[8]);
                return Quaternion{(mat[5] - mat[7]) / s, 0.25 * s, (mat[3] + mat[1]) / s, (mat[6] + mat[2]) / s};
            }
            if (mat[4] > mat[8])
            {
                const double s = 2. * ::std::sqrt(1. + mat[4] - mat[0] - mat[8]);
                return Quaternion{(mat[6] - mat[2]) / s, (mat[3] + mat[1]) / s, 0.25 * s, (mat[7] + mat[5]) / s};
            }

            const double s = 2. * ::std::sqrt(1. + mat[8] - mat[0] - mat[4]);
            return Quaternion{(mat[1] - mat[3]) / s, (mat[6] + mat[2]) / s, (mat[7] + mat[5]) / s, 0.25 * s};
        }

        inline Quaternion Quaternion::fromAxisAngle(const double* axis, double theta)
        {
            const Vector3 axis_v{axis[0], axis[1], axis[2]};
            const double axisNorm = axis_v.norm();
            if (axisNorm == 0.)
            {
                return identity();
            }

            const double halfTheta = 0.5 * theta;
            const double s = ::std::sin(halfTheta) / axisNorm;
            const Vector3 v = axis_v * s;
            return Quaternion{::std::cos(halfTheta), v.x, v.y, v.z};
        }

        inline Quaternion Quaternion::operator+(const Quaternion &q) const
        {
            return Quaternion{w + q.w, x + q.x, y + q.y, z + q.z};
        }

        inline Quaternion Quaternion::operator-(const Quaternion &q) const
        {
            return Quaternion{w - q.w, x - q.x, y - q.y, z - q.z};
        }

        inline Quaternion Quaternion::operator*(const Quaternion &q) const
        {
            return Quaternion{
                w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
        }

        inline Quaternion Quaternion::operator*(double s) const
        {
            return Quaternion{w * s, x * s, y * s, z * s};
        }

        inline Quaternion Quaternion::operator/(double s) const
        {
            const double r = 1. / s;
            return Quaternion{w * r, x * r, y * r, z * r};
        }

        inline const Quaternion Quaternion::operator-() const { return Quaternion{-w, -x, -y, -z}; }

        template <typename T>
        inline Quaternion operator*(const T s, const Quaternion &q)
        {
            return Quaternion{s * q.w, s * q.x, s * q.y, s * q.z};
        }

        inline Quaternion &Quaternion::operator+=(const Quaternion &q)
        {
            w += q.w;
            x += q.x;
            y += q.y;
            z += q.z;
            return *this;
        }

        inline Quaternion &Quaternion::operator-=(const Quaternion &q)
        {
            w -= q.w;
            x -= q.x;
            y -= q.y;
            z -= q.z;
            return *this;
        }

        inline Quaternion &Quaternion::operator*=(const Quaternion &q)
        {
            *this = *this * q;
            return *this;
        }

        inline Quaternion &Quaternion::operator*=(const double &s)
        {
            w *= s;
            x *= s;
            y *= s;
            z *= s;
            return *this;
        }

        inline Quaternion &Quaternion::operator/=(const double &s)
        {
            w /= s;
            x /= s;
            y /= s;
            z /= s;
            return *this;
        }

        inline bool Quaternion::operator==(const Quaternion &q) const
        {
            return w == q.w && x == q.x && y == q.y && z == q.z;
        }

        inline bool Quaternion::operator!=(const Quaternion &q) const { return !(*this == q); }

        inline double Quaternion::norm() const { return ::std::sqrt(w * w + x * x + y * y + z * z); }

        inline double Quaternion::norm2() const { return w * w + x * x + y * y + z * z; }

        inline double dot(const Quaternion &p, const Quaternion &q)
        {
            return p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z;
        }

        inline Quaternion Quaternion::conjugate() const { return Quaternion{w, -x, -y, -z}; }

        inline Quaternion Quaternion::inverse() const
        {
            return conjugate() / norm2();
        }

        inline Quaternion Quaternion::normalize() const
        {
            const double r = 1. / ::std::sqrt(w * w + x * x + y * y + z * z);
            return *this * r;
        }

        inline Quaternion Quaternion::normalizeCutoff(double mag) const
        {
            double len = ::std::sqrt(w * w + x * x + y * y + z * z);
            if (len <= mag)
            {
                len = 1.;
            }
            const double r = 1. / len;
            return *this * r;
        }

        inline Quaternion Quaternion::unit() const { return normalize(); }

        inline void Quaternion::to_matrix(double *mat) const
        {
            const Quaternion q = normalize();
            const double xx = q.x * q.x;
            const double yy = q.y * q.y;
            const double zz = q.z * q.z;
            const double xy = q.x * q.y;
            const double xz = q.x * q.z;
            const double yz = q.y * q.z;
            const double wx = q.w * q.x;
            const double wy = q.w * q.y;
            const double wz = q.w * q.z;

            mat[0] = 1. - 2. * (yy + zz);
            mat[1] = 2. * (xy + wz);
            mat[2] = 2. * (xz - wy);
            mat[3] = 2. * (xy - wz);
            mat[4] = 1. - 2. * (xx + zz);
            mat[5] = 2. * (yz + wx);
            mat[6] = 2. * (xz + wy);
            mat[7] = 2. * (yz - wx);
            mat[8] = 1. - 2. * (xx + yy);
        }

        inline void Quaternion::to_euler(double &psi, double &theta, double &phi) const
        {
            const Quaternion q = normalize();

            const double sinrCosp = 2. * (q.w * q.x + q.y * q.z);
            const double cosrCosp = 1. - 2. * (q.x * q.x + q.y * q.y);
            phi = ::std::atan2(sinrCosp, cosrCosp);

            const double sinp = 2. * (q.w * q.y - q.z * q.x);
            const double halfPi = 1.57079632679489661923;
            if (sinp >= 1.)
            {
                theta = halfPi;
            }
            else if (sinp <= -1.)
            {
                theta = -halfPi;
            }
            else
            {
                theta = ::std::asin(sinp);
            }

            const double sinyCosp = 2. * (q.w * q.z + q.x * q.y);
            const double cosyCosp = 1. - 2. * (q.y * q.y + q.z * q.z);
            psi = ::std::atan2(sinyCosp, cosyCosp);
        }


        inline void Quaternion::rotate(const double *v, double *out) const
        {
            const Quaternion q = normalize();
            const Vector3 qv{q.x, q.y, q.z};
            const Vector3 p{v[0], v[1], v[2]};
            const Vector3 t = 2.0 * cross(qv, p);
            const Vector3 r = p + q.w * t + cross(qv, t);
            out[0] = r.x;
            out[1] = r.y;
            out[2] = r.z;
        }

        inline bool Quaternion::isFinite() const
        {
            return ::std::isfinite(w) && ::std::isfinite(x) && ::std::isfinite(y) && ::std::isfinite(z);
        }

        inline bool Quaternion::isDefined() const
        {
            return (!::std::isnan(w)) && (!::std::isnan(x)) && (!::std::isnan(y)) && (!::std::isnan(z));
        }

        inline std::ostream &operator<<(std::ostream &output, const Quaternion &q)
        {
            output << "<" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ">";
            return output;
        }

        inline std::istream &operator>>(std::istream &input, Quaternion &q)
        {
            double w, x, y, z;
            input >> w >> x >> y >> z;
            q = Quaternion{w, x, y, z};
            return input;
        }
} // namespace SolidMesh

namespace std
{
    inline std::size_t hash<SolidMesh::Quaternion>::operator()(const SolidMesh::Quaternion &q) const
    {
        return std::hash<double>{}(q.w) ^
               (std::hash<double>{}(q.x) + (std::hash<double>{}(q.x) << 2)) ^
               (std::hash<double>{}(q.y) + (std::hash<double>{}(q.y) << 4)) ^
               (std::hash<double>{}(q.z) + (std::hash<double>{}(q.z) << 6));
    }

    inline std::string to_string(SolidMesh::Quaternion q)
    {
        ostringstream output;
        output << q;
        return output.str();
    }

} // namespace std
