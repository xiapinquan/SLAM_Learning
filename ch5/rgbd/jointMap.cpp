#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pangolin/pangolin.h>
#include <vector>
#include <Eigen/Core>
#include <unistd.h>
#include <cv.hpp>
#include <boost/format.hpp>
#include <sophus/se3.hpp>

using namespace Eigen;
using namespace std;

typedef Matrix<double,6,1> Vector6d;

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

/*
 * rgbg相机 进行5张图片的点云合成
 * */
int main(int argc,char ** argv){
    //定义内参
    double fx = 518.0 , fy = 519.0 , cx = 325.5 , cy = 253.5;
    //定义深度缩放系数
    double depthScale = 1000.0;

    //位姿信息、读取颜色图片、深度图片
    ifstream fin("../rgbd/pose.txt");
    if(!fin){
        cerr<<"can not find this pose.txt!!!"<<endl;
        return 0;
    }

    vector<cv::Mat> color_list,depth_list;
    vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d>> SE3_list;

    for(int i=0;i<5;i++){
        boost::format fmt("../rgbd/%s/%d.%s");
        cv::Mat color = cv::imread((fmt %"color" %(i+1) %"png").str());
        cv::Mat depth = cv::imread((fmt %"depth" %(i+1) %"pgm").str(),-1);   //-1表示原图读取
        color_list.push_back(color);
        depth_list.push_back(depth);

        double data[7];
        //从pose中读取参数
        for(auto &d:data){
            fin >> d;
        }

        //生成pose容器
        Sophus::SE3d pose = Sophus::SE3d(Quaterniond(data[6],data[3],data[4],data[5])
                ,Vector3d(data[0],data[1],data[2]));

        SE3_list.push_back(pose);
    }
    cout<<"depth.size =  "<<color_list.size()<<endl;
    cout<<"depth[1].size =  "<<color_list[1].size<<endl;

    //遍历五张图片
      //遍历每个像素点
        //添加每个点到点云容器中
    vector<Vector6d,Eigen::aligned_allocator<Vector6d>> point_cloud;
    point_cloud.reserve(1000000);
    double x,y;
    unsigned int d;
    Vector3d p;
    Vector6d point;
    for(int i=0;i<5;i++){
        cout<<"图像转换中："<<i+1<<endl;
        cv::Mat color = color_list[i];
        cv::Mat depth = depth_list[i];
        Sophus::SE3d T = SE3_list[i];
        cout<<" rows =  "<<color_list[i].rows<<endl;
        cout<<" cols =  "<<color_list[i].cols<<endl;

        cout<<" color.step =  "<<color.step<<endl;   //step = step[0]
        cout<<" color.step[0] =  "<<color.step[0]<<endl;
        cout<<" color.step[1] =  "<<color.step[1]<<endl;



        for(int v=0;v<color.rows;v++){
            for(int u=0;u<color.cols;u++){
                //cout<<"( "<<u<<" , "<<v<<")"<<endl;
                //提取相机坐标系坐标
                d = depth.at<unsigned short>(v,u);  //depth.ptr<unsigned short>(v)[u];
                if(d == 0){
                    continue;
                }
                double dd= double(d)/depthScale;
                x = (u-cx)*dd/fx;
                y = (v-cy)*dd/fy;
                Vector3d p(x,y,dd);

                //计算世界坐标系坐标
                Vector3d point_w;
                point_w = T*p;

                //加入到点云容器中
                point.head<3>() = point_w;
                point[5] = color.data[v*color.step + u* color.channels()];
                point[4] = color.data[v*color.step + u* color.channels()+1];
                point[3] = color.data[v*color.step + u* color.channels()+2];

                point_cloud.push_back(point);
            }
        }

    }
    cout<<"point cloud.size = "<<point_cloud.size()<<endl;
    //显示点云
    showPointCloud(point_cloud);
    return 0;
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

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
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
