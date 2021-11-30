////
//// Created by book on 2021/11/25.
////
//
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Dense"


#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;

int main (int argc,char ** argv){

    Matrix<float,5,5> matrix_55 = Matrix<float,5,5>::Constant(0.5);
    cout<<"here is  matrix_55:\n"<<matrix_55<<endl;

    Matrix3f marix_33 = Matrix3f::Identity();
    cout<<"here is  marix_33:\n"<<marix_33<<endl;

    cout<<"here is  matrix_55_block:\n"<< matrix_55.block(0,0,3,3) <<endl;

    matrix_55.block(0,0,3,3) = marix_33;
    cout<<"here is  matrix_55:\n"<<matrix_55<<endl;




    return 0;
}

//#include <iostream>
//#include "/usr/include/eigen3/Eigen/Dense"
//using namespace std;
//using namespace Eigen;
//
//int main()
//{
//    Matrix4d matrix_44 = Matrix4d::Constant(0.5);
//    cout << "Here is a matrix:\n" << matrix_44 << endl;
//    Matrix3d matrix_33 = Matrix3d::Constant(0.1);
//    matrix_44.block(0,0,3,3) = matrix_33;
//    cout << "左上角３×３的块取出来赋值为Matrix3_3\n" << matrix_44 << endl;
//    return 0;
//}