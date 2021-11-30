//
// Created by xia on 2021/11/18.
//
#include "iostream"
#include "Eigen/Core"
#include "Eigen/Geometry"    //提供了各种旋转和平移的表示

using namespace std;
using namespace Eigen;   //

int main(int argc,char **argv){   //
    cout << "===== ch3 begin=====" << endl;

//====================旋转矩阵与旋转向量====================
    Matrix3d rotation_matrix = Matrix3d::Identity();  //identity matrix 就是单位阵
    AngleAxisd rotation_vector(M_PI/4,Vector3d(0, 0,1));   //沿Z轴旋转45°

    cout.precision(3);  //控制输出的有效数字位数
    rotation_matrix = rotation_vector.toRotationMatrix();
    //rotation_vector.matrix()  //这两个函数一样的，都是把旋转向量转为旋转矩阵
    cout << ""<<rotation_matrix<<endl;

    Vector3d point(1,0,0); //定义一个点
    Vector3d point_rotated = rotation_matrix * point;
    cout<<"point = ("<<point.transpose()<<")   ,point rotated:("<<point_rotated.transpose()<<")"<<endl;
    point_rotated = rotation_vector * point;  //进行运算符重载了，所以可以计算
    cout<<"point = ("<<point.transpose()<<")   ,point rotated:("<<point_rotated.transpose()<<")"<<endl;

//====================欧拉角====================
    Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);   //按照zyx顺序
    //旋转矩阵=>欧拉角
    cout<<"yaw pinth roll : "<<euler_angles.transpose()<<endl;

//====================欧式变换====================
    //欧式变换矩阵使用 Isometry
    Isometry3d T = Isometry3d::Identity();  //虽然是3d，但是是4维的
    T.rotate(rotation_vector); //按照rotation_vector进行旋转
    T.pretranslate(Vector3d(1,3,4));
    cout<<"Transformed matrix : \n"<<T.matrix()<<endl;

    //变换矩阵进行坐标变换
    point_rotated = T * point;
    cout<<"point transformed:"<<point_rotated.transpose()<<endl;

//====================四元数====================
    Quaterniond q(rotation_vector);
    q = Quaterniond(rotation_matrix); //也可以把旋转矩阵赋值给四元数
    cout<<"Quaterniond from rotation_vector is:"<<q.coeffs().transpose()<<endl;
    //这里要主要coeffs的顺序是(x,y,z,w),w为实部,前三者为虚部

    //使用四元数旋转一个坐标
    point_rotated = q * point; //这里进行了运算符重载，数学上是 p'=qpq^-1
    cout<<"Quaterniond point transformed 1:"<<point_rotated.transpose()<<endl;
    //或者不使用运算符重载，直接计算
    cout<<"Quaterniond point transformed 2:"
    <<(q*Quaterniond(0,1,0,0)*q.inverse()).coeffs().transpose()<<endl;

    //对于仿摄变换和摄影变换使用
    //Eigen::Affine3d 和  Eigen::projective3d
    return 0;
}
