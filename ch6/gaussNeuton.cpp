//
// Created by book on 2021/12/15.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>   //进行矩阵的运算，求逆、特征值等

using namespace std;
using namespace Eigen;

/*
 * 利用gaussNeuton法进行曲线的拟合，求出曲线的系数  y = exp(ax^2 + bx +c) + w
 * 求出 a b c 系数， w为噪声，服从w~（0,sigma^2）
 * */
int main(int argc,char **argv){
    cout<<"===========ch6:gaussNeuton"<<endl;

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

    int epoch = 100;
    double cost = 0;
    double lastCost = 0;

    Matrix3d H;
    Vector3d g;
    //迭代循环 次数=epch

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(int i=0 ; i<epoch ; i++) {
        //初始化H，g
        H = Matrix3d::Zero();
        g = Vector3d::Zero();

        //遍历每一个点
        for(int j=0 ; j<N ; j++){
            //求出雅克比矩阵J
            Vector3d J;
            J[0] = -x_data.at(j)*x_data.at(j)*exp(ae*x_data.at(j)*x_data.at(j) + be*x_data.at(j) + ce);
            J[1] = -x_data.at(j)*exp(ae*x_data.at(j)*x_data.at(j) + be*x_data.at(j) + ce);
            J[2] = -exp(ae*x_data.at(j)*x_data.at(j) + be*x_data.at(j) + ce);

            //H+=J*J^T   g+=-J*e
            H += J*J.transpose();
            double error =  y_data.at(j) - exp(ae*x_data.at(j)*x_data.at(j) + be*x_data.at(j) + ce);
            g += -J*error;

            //cost++
            cost += error*error;
        }
        //求解，使用choleshy
        Vector3d dx = H.ldlt().solve(g);
        //dx = H.colPivHouseholderQr().solve(g);   //或者用QR分解法
        if(isnan(dx[0])){
            cout<<"result is nan!"<<endl;
            break;
        }

        //判断当前epoch的cost 没有 >= lastCost
        if(i > 0  && cost >= lastCost){
            cout<<"now epoch :"<<i<<"   ,cost :"<<cost<<"   ,lastCost :"<<lastCost<<"  , cost >= last cost,break!!!"<<endl;
            break;
        }
        lastCost = cost;

        //更新 a b c系数
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        cout<<"now epoch :"<<i<<"   ,cost :"<<cost<<"   ,update dx :"<<dx.transpose()<<endl;
        cout<<"estimated a b c = "<<ae<<"  "<<be<<"  "<<ce<<endl<<endl;
        cost = 0;
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =  chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"time used :"<<time_used.count()<<" 秒"<<endl;
    cout<<"estimated a b c = "<<ae<<"  "<<be<<"  "<<ce<<endl<<endl;

    return 0;
}