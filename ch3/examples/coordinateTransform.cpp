//
// Created by book on 2021/11/20.
//
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc,char **argv){

    Vector3d point(0.5,0,0.2);
    Quaterniond q1(0.35,0.2,0.3,0.1);
    Quaterniond q2(-0.5,0.4,-0.1,0.2);

    //一定要归一化
    q1.normalize();
    q2.normalize();

    Vector3d t1(0.3,0.1,0.1);
    Vector3d t2(-0.1,0.5,0.3);

    Isometry3d T_r1_w(q1);
    T_r1_w.pretranslate(t1);

    Isometry3d T_r2_w(q2);
    T_r2_w.pretranslate(t2);

    Vector3d point_rotated = T_r2_w * T_r1_w.inverse() * point;
    cout<<"point rotated : "<<point_rotated.transpose()<<endl;


    return 0;
}