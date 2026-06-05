#include "quaternion.h"
namespace SolidMesh
{
    inline ShRep::ShRep()
    {
        for (int i = 9; i--; coeff[i] = 0.)
            ;
    };

    inline ShRep::ShRep(double x0, double x1, double x2, double x3, double x4, double x5, double x6, double x7, double x8)

    {
        coeff[0] = x0;
        coeff[1] = x1;
        coeff[2] = x2;
        coeff[3] = x3;
        coeff[4] = x4;
        coeff[5] = x5;
        coeff[6] = x6;
        coeff[7] = x7;
        coeff[8] = x8;
    }

    inline void ShRep::mult9(const double M[9][9])
    { // M*coeff, coeff being column vector
        ShRep copy(*this);
        for (int row = 0; row < 9; row++)
        {
            coeff[row] = 0;
            for (int column = 0; column < 9; column++)
            {
                coeff[row] += M[row][column] * copy[column];
            }
        }
    }

    inline void ShRep::Rz(double a)
    {
        double M[9][9] = {{cos(4. * a), 0, 0, 0, 0, 0, 0, 0, sin(4. * a)},
                          {0, cos(3. * a), 0, 0, 0, 0, 0, sin(3. * a), 0},
                          {0, 0, cos(2. * a), 0, 0, 0, sin(2. * a), 0, 0},
                          {0, 0, 0, cos(a), 0, sin(a), 0, 0, 0},
                          {0, 0, 0, 0, 1, 0, 0, 0, 0},
                          {0, 0, 0, -sin(a), 0, cos(a), 0, 0, 0},
                          {0, 0, -sin(2. * a), 0, 0, 0, cos(2. * a), 0, 0},
                          {0, -sin(3. * a), 0, 0, 0, 0, 0, cos(3. * a), 0},
                          {-sin(4. * a), 0, 0, 0, 0, 0, 0, 0, cos(4. * a)}};
        mult9(M);
    }

    inline void ShRep::Rx90()
    {
        // just in case, inverse of the matrix is equal to its transpose
        double M[9][9] = {{0, 0, 0, 0, 0, std::sqrt(14.) / 4., 0, -std::sqrt(2.) / 4., 0},
                          {0, -3. / 4., 0, std::sqrt(7.) / 4., 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 0, std::sqrt(2.) / 4., 0, std::sqrt(14.) / 4., 0},
                          {0, std::sqrt(7.) / 4., 0, 3. / 4., 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 3. / 8., 0, std::sqrt(5.) / 4., 0, std::sqrt(35.) / 8.},
                          {-std::sqrt(14.) / 4., 0, -std::sqrt(2.) / 4., 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, std::sqrt(5.) / 4., 0, 1. / 2., 0, -std::sqrt(7.) / 4.},
                          {std::sqrt(2.) / 4., 0, -std::sqrt(14.) / 4., 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, std::sqrt(35.) / 8., 0, -std::sqrt(7.) / 4., 0, 1. / 8.}};
        mult9(M);
    }

    inline void ShRep::RxMinus90()
    {
        // just in case, inverse of the matrix is equal to its transpose
        double M[9][9] = {{0, 0, 0, 0, 0, -std::sqrt(14.) / 4., 0, std::sqrt(2.) / 4., 0},
                          {0, -3. / 4., 0, std::sqrt(7.) / 4., 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 0, -std::sqrt(2.) / 4., 0, -std::sqrt(14.) / 4., 0},
                          {0, std::sqrt(7.) / 4., 0, 3. / 4., 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 3. / 8., 0, std::sqrt(5.) / 4., 0, std::sqrt(35.) / 8.},
                          {std::sqrt(14.) / 4., 0, std::sqrt(2.) / 4., 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, std::sqrt(5.) / 4., 0, 1. / 2., 0, -std::sqrt(7.) / 4.},
                          {-std::sqrt(2.) / 4., 0, std::sqrt(14.) / 4., 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, std::sqrt(35.) / 8., 0, -std::sqrt(7.) / 4., 0, 1. / 8.}};
        mult9(M);
    }

    // 当前球谐函数绕y旋转alpha弧度
    inline void ShRep::Ry(double alpha)
    {
        Rx90();
        Rz(alpha);
        RxMinus90();
    }

    // 当前球谐函数绕x旋转alpha弧度
    inline void ShRep::Rx(double alpha)
    {
        Ry(-M_PI / 2);
        Rz(alpha);
        Ry(M_PI / 2);
    }

    inline void ShRep::Rot(const double *quaternion)
    {
        Quaternion rv{quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
        double alpha, beta, gamma;
        rv.to_euler(alpha, beta, gamma);
        Rx(alpha);
        Ry(beta);
        Rz(gamma);
    }

    inline ShRep ShRep::Ex()
    {
        return ShRep(-sqrt(2.) * coeff[7],
                     -sqrt(2.) * coeff[8] - sqrt(3.5) * coeff[6],
                     -sqrt(3.5) * coeff[7] - sqrt(4.5) * coeff[5],
                     -sqrt(4.5) * coeff[6] - sqrt(10.) * coeff[4],
                     sqrt(10.) * coeff[3],
                     sqrt(4.5) * coeff[2],
                     sqrt(3.5) * coeff[1] + sqrt(4.5) * coeff[3],
                     sqrt(2.) * coeff[0] + sqrt(3.5) * coeff[2],
                     sqrt(2.) * coeff[1]);
    }

    inline ShRep ShRep::Ey()
    {
        return ShRep(sqrt(2.) * coeff[1],
                     -sqrt(2.) * coeff[0] + sqrt(3.5) * coeff[2],
                     -sqrt(3.5) * coeff[1] + sqrt(4.5) * coeff[3],
                     -sqrt(4.5) * coeff[2],
                     -sqrt(10.) * coeff[5],
                     -sqrt(4.5) * coeff[6] + sqrt(10.) * coeff[4],
                     -sqrt(3.5) * coeff[7] + sqrt(4.5) * coeff[5],
                     -sqrt(2.) * coeff[8] + sqrt(3.5) * coeff[6],
                     sqrt(2.) * coeff[7]);
    }

    inline ShRep ShRep::Ez()
    {
        return ShRep(4 * coeff[8],
                     3 * coeff[7],
                     2 * coeff[6],
                     coeff[5],
                     0,
                     -coeff[3],
                     -2 * coeff[2],
                     -3 * coeff[1],
                     -4 * coeff[0]);
    }

    inline double ShRep::operator*(const ShRep &other)
    {
        double res = 0;
        for (int d = 0; d < 9; d++)
            res += coeff[d] * other.coeff[d];
        return res;
    }

    inline double ShRep::norm()
    {
        return sqrt((*this) * (*this));
    }

    inline void ShRep::normalize()
    {
        double s = norm();
        for (int d = 0; d < 9; d++)
            coeff[d] /= s;
    }

    
    inline void ShRep::project(ShRep query, double *W, double grad_threshold, double dot_threshold)
    {
        // 初始化一些姿态
        ShRep init_harmonics[5] = {
            ShRep(0, 0, 0, 0, std::sqrt(7. / 12.), 0, 0, 0, std::sqrt(5. / 12.)),   // 参考Frame
            ShRep(0, 0, 0, 0, -0.190941, 0, -0.853913, 0, 0.484123),                // 绕x轴旋转pi/4后的Frame（SO3上的鞍点）
            ShRep(0, 0, 0, 0, -0.190941, 0, 0.853913, 0, 0.484123),                 // 绕y轴旋转pi/4后的Frame（SO3上的鞍点）
            ShRep(0, 0, 0, 0, 0.763763, 0, 0, 0, -0.645497),                        // 绕z轴旋转pi/4后的Frame（SO3上的鞍点）
            ShRep(0, 0, -0.853913, 0, -0.190941, 0, 0, 0, -0.484123)};              // 绕面对角线(1,1,0)轴旋转pi/4后的Frame（SO3上的鞍点）

        Quaternion init_rot[5] = {
            Quaternion{0, 1, 0, 0},                                         // 不旋转
            Quaternion{0.78539816339, 1, 0, 0},                             // 绕x轴旋转pi/4
            Quaternion{0.78539816339, 0, 1, 0},                             // 绕y轴旋转pi/4
            Quaternion{0.78539816339, 0, 0, 1},                             // 绕z轴旋转pi/4
            Quaternion{0.78539816339, 0.70710678118, 0.70710678118, 0}};    // 绕面对角线(1,1,0)轴旋转pi/4

        ShRep v;
        double dot = -1.;
        query.normalize();

        // 从5个初始旋转中选一个与query最接近的作为迭代起点（最大化内积等价于最小化L2距离）
        for (int i = 0; i < 5; i++)
        {
            double tdot = init_harmonics[i] * query;
            if (tdot > dot)
            {
                dot = tdot;
                W = init_rot[i];
                v = init_harmonics[i];
            }
        }

        int cnt = 0;
        double olddot = dot;
        // 
        while (cnt < 10000)
        {
            Vector3 grad{query * v.Ex(), query * v.Ey(), query * v.Ez()};   // 论文中能量的梯度，三个分量是欧拉角
            if (grad.norm() < grad_threshold)
                break;
            grad = (1. / 8.) * grad; // empirically found constant
            v.Rx(grad[0]);
            v.Ry(grad[1]);
            v.Rz(grad[2]);
            W = Quaternion{grad[0], 1, 0, 0} * (*W);
            W = Quaternion{grad[1], 0, 1, 0} * (*W);
            W = Quaternion{grad[2], 0, 0, 1} * (*W);
            cnt++;
            dot = v * query;
            if (dot - olddot < dot_threshold)
                break;
            olddot = dot;
        }
        if (cnt == 10000)
            std::cerr << "[error] SH projection infinite loop protection" << std::endl;
    }
}
