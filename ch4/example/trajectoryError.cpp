//
// Created by book on 2021/12/1.
//
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>


using namespace std;
using namespace Eigen;

string trajectory_estimated = "../example/estimated.txt";
string trajectory_groundtruth = "../example/groundtruth.txt";

typedef vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

TrajectoryType readTrajectory(string file_path);
void drawTrajectory(TrajectoryType trajectory_real,TrajectoryType trajectory_est);

int main(int argc,char ** argv) {
    cout<<"===== this is ch4:example trajectoryError ! ====="<<endl<<endl;

    TrajectoryType trajectory_est = readTrajectory(trajectory_estimated);
    TrajectoryType trajectory_real = readTrajectory(trajectory_groundtruth);

    double rmse = 0;
    double error = 0;
    for(int i=0;i<trajectory_est.size();i++){
        error = (trajectory_est[i].inverse()*trajectory_real[i]).log().norm();
        rmse += error*error;
    }
    rmse/=(double)trajectory_est.size();
    rmse = sqrt(rmse);
    cout<<"rmse is : "<<rmse<<endl;

    drawTrajectory(trajectory_real,trajectory_est);
    return 0;
}

TrajectoryType readTrajectory(string file_path){
    TrajectoryType trajectory;
    ifstream fin(file_path);
    if (!fin) {
        cout << "cannot find trajectory file at " << file_path << endl;
        return trajectory;
    }

    Sophus::SE3d SE3;
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        SE3 = Sophus::SE3d(Quaterniond(qw, qx, qy, qz),Vector3d(tx, ty, tz));
        trajectory.push_back(SE3);
    }
    cout << "read trajectory file : "<<file_path<<", Its size = "<<trajectory.size()<<endl;
    return trajectory;
}

void drawTrajectory(TrajectoryType trajectory_real,TrajectoryType trajectory_est) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        // 画出连线
        for (size_t i = 0; i < trajectory_real.size(); i++) {
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = trajectory_real[i], p2 = trajectory_real[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < trajectory_est.size(); i++) {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = trajectory_est[i], p2 = trajectory_est[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

