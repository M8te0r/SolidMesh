#pragma once
#include <cmath>

namespace SolidMesh
{
    /**
     * @brief 使用l=4的球谐基及其9个系数来表示Frame的立方八面体对称性
     * 每个Frame可以视为参考Frame旋转后的结果
     * 参考Frame: \tilde{F}=\sqrt{7/12}* Y_4^0 + \sqrt{5/12} * Y_4^4
     * 假设l=4的球谐基为行向量B={Y_4^-4, Y_4^-3,...,Y_4^4}，其参数为a={a_-4, a_-3,...,a_4}
     * 则参考Frame为：\tilde{F}=B*\tilde{a}，\tilde{a}={0,0,0,0,\sqrt{7/12},0,0,0,\sqrt{5/12}}
     * 其余Frame为：F=Ba，其参数可以表示为参考Frame参数在球谐参数基参数空间的旋转：a=R_B \tilde{a}, R_B为9*9的矩阵
     */
    struct ShRep
    {
        // 使用l=4的球谐基函数下的SO3唯一表示，9维向量
        double coeff[9];

        ShRep();
        ShRep(double x0, double x1, double x2, double x3, double x4, double x5, double x6, double x7, double x8);
        double &operator[](int i) { return coeff[i]; }

        // 球谐参数左乘9*9旋转矩阵(col-major)，即旋转球谐函数
        void mult9(const double M[9][9]);

        // 当前球谐函数绕z旋转a弧度
        void Rz(double a);

        // 当前球谐函数绕x逆时针旋转90度
        void Rx90();

        // 当前球谐函数绕x顺时针旋转90度
        void RxMinus90();

        // 当前球谐函数绕y旋转alpha弧度
        void Ry(double alpha);

        // 当前球谐函数绕x旋转alpha弧度
        void Rx(double alpha);

        // 根据四元数quaternion旋转当前球谐函数
        void Rot(const double *quaternion);

        /*
            参数空间的旋转线性化为如下表示：
            R_B^x(\alpha)=I_{9\times 9} + \alpha * E_B^x + O(|\alpha|)
            R_B^y(\beta)=I_{9\times 9} + \beta * E_B^y + O(|\beta|)
            R_B^z(\gamma)=I_{9\times 9} + \gamma * E_B^z + O(|\gamma|)
         */

        
        // 旋转对
        // \partial{R_B(\theta_x,\theta_y,\theta_z)} / \partial{\theta_x} \times \tilde{a}, theta_x=theta_y=theta_z=0
        ShRep Ex();
        ShRep Ey();
        ShRep Ez();

        double operator*(const ShRep &other);

        double norm();

        void normalize();

        // 给定一个目标球谐表示query，寻找一个旋转quaternion，使得参考标架旋转后与query尽可能接近
        static void project(ShRep query, double *quaternion, double grad_threshold, double dot_threshold);
    };
}

#include "sh_rep.inl"