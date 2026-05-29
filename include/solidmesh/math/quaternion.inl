namespace SolidMesh
{
        inline Quaternion Quaternion::fromRotationMatrix(const double m[9])
        {
            const double trace = m[0] + m[4] + m[8];
            if (trace > 0.)
            {
                const double s = 2. * ::std::sqrt(trace + 1.);
                return Quaternion{0.25 * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s};
            }
            if (m[0] > m[4] && m[0] > m[8])
            {
                const double s = 2. * ::std::sqrt(1. + m[0] - m[4] - m[8]);
                return Quaternion{(m[7] - m[5]) / s, 0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s};
            }
            if (m[4] > m[8])
            {
                const double s = 2. * ::std::sqrt(1. + m[4] - m[0] - m[8]);
                return Quaternion{(m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s};
            }

            const double s = 2. * ::std::sqrt(1. + m[8] - m[0] - m[4]);
            return Quaternion{(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s};
        }

        inline Quaternion Quaternion::fromAxisAngle(const double axis[3], double theta)
        {
            const double axisNorm = ::std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            if (axisNorm == 0.)
            {
                return identity();
            }

            const double halfTheta = 0.5 * theta;
            const double s = ::std::sin(halfTheta) / axisNorm;
            return Quaternion{::std::cos(halfTheta), axis[0] * s, axis[1] * s, axis[2] * s};
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
        inline double norm(const Quaternion &q) { return q.norm(); }

        inline double Quaternion::norm2() const { return w * w + x * x + y * y + z * z; }
        inline double norm2(const Quaternion &q) { return q.norm2(); }

        inline double dot(const Quaternion &p, const Quaternion &q)
        {
            return p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z;
        }

        inline Quaternion Quaternion::conjugate() const { return Quaternion{w, -x, -y, -z}; }
        inline Quaternion conjugate(const Quaternion &q) { return q.conjugate(); }

        inline Quaternion Quaternion::inverse() const
        {
            return conjugate() / norm2();
        }
        inline Quaternion inverse(const Quaternion &q) { return q.inverse(); }

        inline Quaternion Quaternion::normalize() const
        {
            const double r = 1. / ::std::sqrt(w * w + x * x + y * y + z * z);
            return *this * r;
        }
        inline Quaternion normalize(const Quaternion &q) { return q.normalize(); }

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
        inline Quaternion normalizeCutoff(const Quaternion &q, double mag) { return q.normalizeCutoff(mag); }

        inline Quaternion Quaternion::unit() const { return normalize(); }
        inline Quaternion unit(const Quaternion &q) { return normalize(q); }

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
        inline void to_matrix(const Quaternion &q, double *mat) { q.to_matrix(mat); }

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
        inline void to_euler(const Quaternion &q, double &psi, double &theta, double &phi)
        {
            q.to_euler(psi, theta, phi);
        }

        inline void Quaternion::rotate(const double *v, double *out) const
        {
            const Quaternion p{0., v[0], v[1], v[2]};
            const Quaternion r = (*this) * p * inverse();
            out[0] = r.x;
            out[1] = r.y;
            out[2] = r.z;
        }
        inline void rotate(const Quaternion &q, const double *v, double *out) { q.rotate(v, out); }

        inline bool Quaternion::isFinite() const
        {
            return ::std::isfinite(w) && ::std::isfinite(x) && ::std::isfinite(y) && ::std::isfinite(z);
        }
        inline bool isfinite(const Quaternion &q) { return q.isFinite(); }

        inline bool Quaternion::isDefined() const
        {
            return (!::std::isnan(w)) && (!::std::isnan(x)) && (!::std::isnan(y)) && (!::std::isnan(z));
        }
        inline bool isDefined(const Quaternion &q) { return q.isDefined(); }

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
