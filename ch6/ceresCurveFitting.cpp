//
// Created by book on 2021/12/19.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;
using namespace Eigen;

//构造代价结构体，用于ceres::Problem构造
struct CostFunctor{
    CostFunctor(double x,double y) : _x(x) , _y(y) {}

    const double _x,_y;
    template<typename T>
    bool operator()(const T *const abc,T *residual)const{
        //y-exp(ax^2 + bx +c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

};

/*
 * 利用ceres库求出拟合曲线，求出曲线的系数  y = exp(ax^2 + bx +c) + w
 * 求出 a b c 系数， w为噪声，服从w~（0,sigma^2）
 * */
int main(int argc,char **argv){
    cout<<"===========ch6: use ceres to fit curve"<<endl;

    double ar = 1.0 , br = 2.0 , cr = 1.0;  //真实参数
    double ae = 2.0 , be = -1.0 , ce = 5.0; //初始化预估参数
    int N = 100;  //数据点
    double w_sigma = 1.0;   //噪声sigma值
    double inv_sigma = 1 / w_sigma;
    cv::RNG rng;  //opencv 随机数产生器

    //生成含有噪声的曲线节点
    vector<double> x_data;
    vector<double> y_data;
    for(int i=0;i<N;i++){
        double x = (double)i/100;
        x_data.push_back(x);
        y_data.push_back((exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma)));
    }

    double abc[3] = {ae,be,ce};

    //构造求解问题
    ceres::Problem problem;
    for(int i=0 ; i<N ; i++){
        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CostFunctor,1,3>
                (new CostFunctor(x_data[i],y_data[i]))
                , nullptr  // 核函数，这里不使用
                ,abc  //传入待估计参数
                );

    }

    //配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  //增量方程的求解方式，choleskey
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;   //优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);  //开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"solve time cost = "<<time_used.count()<<" seconds"<<endl;
    cout<<summary.BriefReport()<<endl;
    cout<<"estimated a,b,c = ";
    for(auto a:abc)
        cout<<a<<" ";
    cout<<endl;


    return 0;
}