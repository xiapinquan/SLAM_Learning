//
// Created by book on 2021/12/6.
//
#include <iostream>
#include <chrono>   //计时所用
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
string image_file = "../imageBasics/distorted.png";

int main(int argc,char **argv){

    cv::Mat img = cv::imread(image_file,0);

    if(img.empty() || img.data == nullptr){
        cerr<<"无法读取到该图片"<<endl;
        return 0;
    }

    int cols = img.cols,rows = img.rows;
    cv::Mat undistorted_img = cv::Mat(rows,cols,CV_8UC1);    //创建去畸变Mat
    cout<<"cols = "<<cols<<"   ,row = "<<rows<<endl;

    //定义畸变参数
    double k1 = -0.28340811 , k2 = 0.07395907 , p1 = 0.00019359 , p2 = 1.76187114e-05;

    //定义内参
    double fx = 458.654 , fy = 457.296 , cx = 367.215 , cy = 248.375;

    for(int u=0 ; u<cols ; u++){
        for(int v=0 ; v<rows ; v++){
            //由图像中的像素(u，v)坐标 计算出归一化的坐标
            double x = (u-cx)/fx;
            double y = (v-cy)/fy;
            double r = sqrt(x*x+y*y);
            //cout<<"x = "<<x<<"   ,y = "<<y<<"   , r = "<<r<<endl;

            //由归一化坐标计算出 畸变归一化坐标
            double x_distort = x*(1 + k1*r*r + k2*pow(r,4)) + 2*p1*x*y + p2*(r*r + 2*x*x);
            double y_distort = y*(1 + k1*r*r + k2*pow(r,4)) + 2*p2*x*y + p1*(r*r + 2*y*y);

            //由畸变坐标计算出 畸变像素坐标
            double u_distort = fx * x_distort+cx;
            double v_distort = fy * y_distort+cy;


            // 映射：畸变像素坐标映射到undistort_img坐标
            if(u_distort <= cols && u_distort >= 0 && v_distort <= rows && v_distort >= 0){
                undistorted_img.at<uchar>(v,u) = img.at<uchar>((int)v_distort,(int)u_distort);
            }else{
                undistorted_img.at<uchar>(v,u) = 0;
            }
        }
    }

    cv::imshow("distorted img",img);
    cv::imshow("undistorted img",undistorted_img);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}