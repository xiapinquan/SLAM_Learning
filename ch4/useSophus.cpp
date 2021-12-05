//
// Created by book on 2021/11/30.
//
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;

int main(int argc,char ** argv){

    cout<<"===== this is ch4:useSophus! ====="<<endl<<endl;
    Matrix3d R = AngleAxisd(M_PI/2,Vector3d(0,0,1)).toRotationMatrix();  //定义旋转矩阵，沿Z轴旋转90°
    Quaterniond q(R);   //或者定义一个四元数
    cout<<"rotation matrix:\n"<<R<<endl<<endl;

    Sophus::SO3d SO3_R(R);  //由旋转矩阵构造的李群
    Sophus::SO3d SO3_q(q);  //由四元数构造的李群    //这里为什么构造出来的李群和旋转矩阵不等？
    cout<<"SO(3) from matrix:\n"<<SO3_R.matrix()<<endl<<endl;
    cout<<"SO(3) from quaternion:\n"<<SO3_q.matrix()<<endl<<endl;

    Vector3d so3 = SO3_q.log();
    cout<<"so(3):\n"<<so3.transpose()<<endl<<endl;  //  获取李代数，结果是【0，0，1.57】 1.57就是3.14/2的结果
    cout<<"so(3) hat :\n"<<Sophus::SO3d::hat(so3)<<endl<<endl;  //向量-->>反对称矩阵
    cout<<"so(3) vee :\n"<<Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose()<<endl<<endl;  //反对称矩阵-->>向量

    //增加扰动模型的更新
    Vector3d update_so3(1e-4,0,0);  //这里是李代数
    Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3)*SO3_R;
    cout<<"SO(3) update :\n"<<SO3_update.matrix()<<endl<<endl;  //更新后的SO3

    //======================================================================================================
    Vector3d t(1,0,0);   //沿X轴平移1
    Sophus::SE3d SE3_Rt(R,t);
    Sophus::SE3d SE3_qt(q,t);
    cout<<"SE(3) from R,t:\n"<<SE3_Rt.matrix()<<endl<<endl;
    cout<<"SE(3) from q,t:\n"<<SE3_Rt.matrix()<<endl<<endl;

    //定义se3这个6维向量
    typedef Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_Rt.log();

    cout<<"se(3) hat :\n"<<Sophus::SE3d::hat(se3)<<endl<<endl;  //向量-->>反对称矩阵
    cout<<"se(3) vee :\n"<<Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose()<<endl<<endl;  //反对称矩阵-->>向量

    //增加更新
    Vector6d se3_update;
    se3_update.setZero();
    se3_update(0,0) = 1e-4d;
    cout<<"se3_update :\n"<<se3_update.transpose()<<endl<<endl;
    Sophus::SE3d update = Sophus::SE3d::exp(se3_update)*SE3_Rt;
    cout<<"update :\n"<<update.matrix()<<endl<<endl;


    return 0;
}