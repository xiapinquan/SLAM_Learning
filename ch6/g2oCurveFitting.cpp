//
// Created by book on 2021/12/20.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <chrono>

//g2o
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;
using namespace Eigen;


// define the node class                模板参数：优化变量的维度，数据类型
class CurveFittingVertex: public g2o::BaseVertex<3,Eigen::Vector3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;   //宏定义 分配内存的方式

    //initual
    void setToOriginImpl() override {
        _estimate << 0,0,0;
    }

    void oplusImpl(const double *update) override {
        _estimate += Vector3d(update);
    }

    bool read(istream &is) override {
        return false;
    }

    bool write(ostream &os) const override {
        return false;
    }

};

//define the edge for the error           模板参数：观测维度、类型、顶点类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x): BaseUnaryEdge(),_x(x){}

    double _x;

    //计算模型误差
    void computeError() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Vector3d  abc = v->estimate();
        _error(0,0) = _measurement - std::exp(abc[0]* _x* _x + abc[1]*_x + abc[2]);
    }

    //计算雅克比矩阵
    void linearizeOplus() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Vector3d  abc = v->estimate();
        double y = exp(abc[0]* _x* _x + abc[1]*_x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x *y;
        _jacobianOplusXi[1] = -_x *y;
        _jacobianOplusXi[2] = -y;

    }

    bool read(istream &is) override {
        return false;
    }

    bool write(ostream &os) const override {
        return false;
    }
};

/*
 * 利用g2o库求出拟合曲线，求出曲线的系数  y = exp(ax^2 + bx +c) + w
 * 求出 a b c 系数， w为噪声，服从w~（0,sigma^2）
 * */
int main(int argc,char **argv){
    cout<<"===========ch6: use ceres to fit curve"<<endl;

    //生成噪声/////////////////////////////////////////////////////////////
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

    ///配置g2o/////////////////////////////////////////////////////////////////
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolverType;  //误差项类型，每个误差项优化变量维度为3，误差值为1
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinerSolverType;  //线性求解器类型

    auto solver = new g2o::OptimizationAlgorithmGaussNewton
            (g2o::make_unique<BlockSolverType>(g2o::make_unique<LinerSolverType>())); //配置梯度下降算法，高斯牛顿
    g2o::SparseOptimizer optimizer;  //定义图类型
    optimizer.setAlgorithm(solver);  //设置求解器算法
    optimizer.setVerbose(true);  //打开调试输出

    //往图中添加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Vector3d(ae,be,ce));
    v->setId(0);
    optimizer.addVertex(v);

    //往图中添加边
    for(int i=0 ; i<N ;i++){
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0,v);  //设置连接的顶点
        edge->setMeasurement(y_data[i]);  //观测数值
        edge->setInformation(Matrix<double,1,1>::Identity() *1 / (w_sigma*w_sigma)); //定义信息矩阵
        optimizer.addEdge(edge);
    }

    //准备开始优化
    cout<<"start optimization!"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =  chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"g2o optimization used time :"<<time_used.count()<<" 秒"<<endl;
    //输出优化值
    Vector3d abc_estimate = v->estimate();
    cout<<"final estimate a b c:"<<abc_estimate.transpose()<<endl;

    return 0;
}
