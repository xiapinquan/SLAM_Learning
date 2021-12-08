//
// Created by book on 2021/12/6.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pangolin/pangolin.h>
#include <vector>
#include <Eigen/Core>
#include <unistd.h>
#include <cv.hpp>

using namespace Eigen;
using namespace std;

string left_file = "../stereoVision/left.png";
string rignt_file = "../stereoVision/right.png";

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc,char ** argv){

    //定义内参、双目相机基线
    double fx = 718.856 , fy =718.856 , cx = 607.1928 , cy = 185.2157;
    double b = 0.573;

    //导入左右 图片
    cv::Mat left = cv::imread(left_file,0);
    cv::Mat right = cv::imread(rignt_file,0);

    //生成视差信息
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0,96,9,8*9*9,32*9*9,1,63,10,100,32
            );
    cv::Mat disparity_sgbm,disparity;
    sgbm->compute(left,right,disparity_sgbm);
    disparity_sgbm.convertTo(disparity,CV_32F,1.0/16.0f);

    //定义点云容器
    vector<Vector4d,Eigen::aligned_allocator<Vector4d>> poses;

    //遍历像素点，根据像素点生成点云
    for(int v = 0; v<left.rows ; v++){
        for(int u = 0; u<left.cols ; u++){

            //关键
            if(disparity.at<float>(v,u) <=10 || disparity.at<float>(v,u) >=96)
                continue;

            Vector4d pose(0,0,0,left.at<uchar>(v,u)/255.0);
            double x = (u-cx)/fx;
            double y = (v-cy)/fy;

            double depth = (fx*b)/disparity.at<float>(v,u);
            //这里根据公式应该是 depth = f*b/d; 如果如果1：1进行像素坐标系变换，那么缩放系数为1，fx=f；

            pose[0] = x * depth;
            pose[1] = y * depth;
            pose[2] = depth;
            poses.push_back(pose);
        }
    }

    //显示disparity
    cv::imshow("disparity",disparity/96);
    cv::waitKey(0);

    //显示点云
    showPointCloud(poses);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}