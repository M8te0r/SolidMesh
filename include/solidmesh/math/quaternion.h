#pragma once
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace SolidMesh
{
        // Components are stored scalar-first: q = w + xi + yj + zk.
        // Keep this as an aggregate type, matching SolidMesh::Vector3 style.
        struct Quaternion
        {
            double w, x, y, z;

            // implicit conversion
            operator const double *() const { return &w; }
            operator double *() { return &w; }

            static Quaternion identity() { return Quaternion{1., 0., 0., 0.}; }
            static Quaternion zero() { return Quaternion{0., 0., 0., 0.}; }
            static Quaternion constant(double c) { return Quaternion{c, c, c, c}; }
            static Quaternion infinity()
            {
                const double inf = ::std::numeric_limits<double>::infinity();
                return Quaternion{inf, inf, inf, inf};
            }
            static Quaternion undefined()
            {
                const double nan = ::std::numeric_limits<double>::quiet_NaN();
                return Quaternion{nan, nan, nan, nan};
            }

            // 根据一个3*3旋转矩阵（column-major）生成quaternion
            static Quaternion fromRotationMatrix(const double *mat); // column-major
            // 创建一个绕着指定旋转轴 axis 旋转 theta 弧度的Quaternion。
            static Quaternion fromAxisAngle(const double* axis, double theta);

            double &operator[](int index) { return (&w)[index]; }
            double operator[](int index) const { return (&w)[index]; }

            Quaternion operator+(const Quaternion &q) const;
            Quaternion operator-(const Quaternion &q) const;
            Quaternion operator*(const Quaternion &q) const;
            Quaternion operator*(double s) const;
            Quaternion operator/(double s) const;
            Quaternion &operator+=(const Quaternion &q);
            Quaternion &operator-=(const Quaternion &q);
            Quaternion &operator*=(const Quaternion &q);
            Quaternion &operator*=(const double &s);
            Quaternion &operator/=(const double &s);
            bool operator==(const Quaternion &q) const;
            bool operator!=(const Quaternion &q) const;
            const Quaternion operator-() const;

            Quaternion conjugate() const;
            Quaternion inverse() const;
            Quaternion normalize() const;
            Quaternion normalizeCutoff(double mag = 0.) const;
            Quaternion unit() const;

            double norm() const;
            double norm2() const;

            // 将quaternion转换为3*3旋转矩阵mat(col-major)
            void to_matrix(double *mat) const; 
            void to_euler(double &psi, double &theta, double &phi) const;

            // 使用当前quaternion旋转向量v，输出out
            void rotate(const double *v, double *out) const;

            bool isFinite() const;
            bool isDefined() const;
        };

        template <typename T>
        Quaternion operator*(const T s, const Quaternion &q);

        ::std::ostream &operator<<(::std::ostream &output, const Quaternion &q);
        ::std::istream &operator>>(::std::istream &input, Quaternion &q);

        double dot(const Quaternion &p, const Quaternion &q);

} // namespace SolidMesh

namespace std
{
    template <>
    struct hash<SolidMesh::Quaternion>
    {
        std::size_t operator()(const SolidMesh::Quaternion &q) const;
    };

    std::string to_string(SolidMesh::Quaternion value);

} // namespace std

#include "quaternion.inl"
