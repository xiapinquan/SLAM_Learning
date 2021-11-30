//
// Created by xia on 2021/11/16.
//
#include <iostream>
#include <Eigen/Core>  //核心
#include <Eigen/Dense>  //稠密矩阵的代数运算(逆、特征值)

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc,char ** argv){
    cout << "===== ch3 begin=====" << endl;

    //矩阵的定义
    Matrix<float,2,3> matrix_f23;  //声明一个2*3的矩阵
    Vector3d v_d31;
    Matrix<float,3,1> v_f31;
    Matrix3d matrix_d33 = Matrix3d::Zero();  //初始化为0
    Matrix<double,Dynamic,Dynamic> matrix_dynamic;  //声明动态大小的矩阵
    MatrixXd matrix_x;         //同样的动态大小

    //矩阵的访问与打印
    v_d31 << 2,3,4;            //直接通过了流输入符进行输入
    v_f31 << 4,5,6;
    cout << v_f31 << endl;    //打印
    //cout << matrix_f23 <<endl;
    for(int i=0;i<2;i++){
        for(int j=0;j<3;j++){
            cout << matrix_f23(i,j)<<"\t";  // \t是tab
        }
        cout<<endl;
    }

    //矩阵和向量相乘
    // Matrix<double,2,1> result = matrix_f23 * v_3d;   //会报错，因为不同类型不能*
    Matrix<double,2,1> result = matrix_f23.cast<double>() * v_d31;   //这里进行显式的转换
    cout<<"result = "<<result<<endl;
    Matrix<float,2,1> result2 = matrix_f23 * v_f31;

    //零矩阵、生成随机数、转置、各元素和、trace、数乘、逆、行列式
    matrix_d33 = Matrix3d::Zero();
    matrix_d33 = Matrix3d::Random();
    cout<<"matrix_d33 random: \n"<<matrix_d33<<endl;
    cout<<"matrix_d33 transpose: \n"<<matrix_d33.transpose()<<endl;  //转置
    cout<<"matrix_d33 sum: "<<matrix_d33.sum()<<endl;
    cout<<"matrix_d33 trace: "<<matrix_d33.trace()<<endl;
    cout<<"matrix_d33 10 times: \n"<<matrix_d33*10<<endl;
    cout<<"matrix_d33 inverse: \n"<<matrix_d33.inverse()<<endl;
    cout<<"matrix_d33 determinant: \n"<<matrix_d33.determinant()<<endl;

    //求特征值
    //实对称矩阵可以保证对角化成功,这里应该是A*A的转置 一定是实对称矩阵
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_d33.transpose() * matrix_d33);
    cout<< "eigen values = "<<eigen_solver.eigenvalues()<<endl;
    cout<< "eigen vectors  = \n"<<eigen_solver.eigenvectors()<<endl;

    //解方程
    //我们求解matrix__NN * x = v_Nd方程  ，N的大小在前面宏定义，矩阵由随机数生成
    //直接求逆是最简单的，但是运算量大，我们一般不采用
    Matrix<double,MATRIX_SIZE,MATRIX_SIZE> matrix_NN =  Matrix<double,MATRIX_SIZE,MATRIX_SIZE>::Random();
    matrix_NN = matrix_NN.transpose()*matrix_NN; //保证半正定矩阵
    Matrix<double,MATRIX_SIZE,1> v_ND = Matrix<double,MATRIX_SIZE,1>::Random();

    clock_t time_set = clock(); //计时
    //直接求逆法
    Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse() * v_ND;
    cout<<"直接求逆法计算时间:"<<1000*(clock()-time_set)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
    cout <<"x = "<<x.transpose()<<endl;

    time_set = clock(); //计时
    //QR分解法
    x = matrix_NN.colPivHouseholderQr().solve(v_ND);
    cout<<"QR分解法计算时间:"<<1000*(clock()-time_set)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
    cout <<"x = "<<x.transpose()<<endl;

    time_set = clock(); //计时
    //cholesky分解法
    x = matrix_NN.ldlt().solve(v_ND);
    cout<<"cholesky分解法计算时间:"<<1000*(clock()-time_set)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
    cout <<"x = "<<x.transpose()<<endl;

    return 0;
}
