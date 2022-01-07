//
// Created by book on 2021/12/29.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;
using namespace Eigen;

void find_feature_matches(
        const Mat &img_1, const Mat &img_2,
        std::vector<KeyPoint> &keypoints_1,
        std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose
);

// BA by gauss-newton
void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose
);

int main(int argc, char **argv) {
    if (argc != 5) {
        cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    //建立3D点
    Mat d1 = imread(argv[3],CV_LOAD_IMAGE_UNCHANGED);
    Mat K = (Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
    vector<Point3d> pts_3d;
    vector<Point2d> pts_2d;
    for(auto m:matches){
        double d = d1.at<unsigned short >((int)keypoints_1[m.queryIdx].pt.y,(int)keypoints_1[m.trainIdx].pt.y);
        if(d == 0){
            continue;
        }
        float dd = d/5000.0;
        Point2d p = pixel2cam(keypoints_1[m.queryIdx].pt,K);
        pts_3d.push_back(Point3d(p.x * dd,p.y * dd,dd));
        pts_2d.push_back(Point2d(keypoints_2[m.trainIdx].pt));
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    Mat r,t;
    solvePnP(pts_3d,pts_2d,K,Mat(),r,t,false);
    Mat R;
    cv::Rodrigues(r,R);
    cout << "R = \n"<<R<<endl;
    cout << "t = \n"<<t.t()<<endl;

    ///////////////////////////////////////////////////////////////
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i) {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    cout << "\ncalling bundle adjustment by gauss newton" << endl;
    Sophus::SE3d pose_gn;
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);

    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3d pose_g2o;
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    return 0;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
            (
                    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}

void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose) {
    typedef Eigen::Matrix<double,6,1> Vector6d;
    const double epoch = 10;
    double fx = K.at<double>(0,0);
    double cx = K.at<double>(0,2);
    double fy = K.at<double>(1,1);
    double cy = K.at<double>(1,2);

    Eigen::Matrix<double,6,6> H;
    Vector6d b;
    double cost,lastCost;
    for(int i=0 ; i<epoch ; ++i){
        cost = 0;
        H.setZero();
        for(int j=0 ; j<points_3d.size() ; j++){
            Vector3d pc = pose * points_3d[j];
            double inv_z = 1.0/pc[2];
            double inv_z2 = inv_z * inv_z;
            Vector2d proj(fx*pc[0]/pc[2]+cx,fy*pc[1]/pc[2]+cy);
            Vector2d e = points_2d[j] - proj;
            cost+=e.squaredNorm();
            Eigen::Matrix<double,2,6> J;
            J<<
            -fx*inv_z , 0 , fx*pc[0]*inv_z2 , fx*pc[0]*pc[1]*inv_z2 , -fx-fx*pc[0]*pc[0]*inv_z2 , fx*pc[1]*inv_z,
            0 , -fy*inv_z , fy*pc[1]*inv_z2 , fy+fy*pc[1]*inv_z2 ,- fy*pc[0]*pc[1]*inv_z2 , -fy*pc[0]*inv_z;

            H += J.transpose()*J;
            b += -J.transpose() * e;
        }
        Vector6d dx = H.ldlt().solve(b);

        if(isnan(dx[0])){
            cout<<"result is nan!"<<endl;
            break;
        }

        if(i > 0 && cost >= lastCost){
            cout<<"cost = "<<cost<<"   ,last cost = "<<lastCost<<endl;
            break;
        }

        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        cout<<"epoch = "<<i<<"   ,cost = "<<cout.precision(12)<<cost<<endl;
        if(dx.norm() <1e-6){
            break;
        }
    }
    cout << "pose = \n"<<pose.matrix()<<endl;
}

class VertexPose:public g2o::BaseVertex<6,Sophus::SE3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(istream &is) override {
        return false;
    }

    bool write(ostream &os) const override {
        return false;
    }

    void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    void oplusImpl(const double *update) override {
        Matrix<double,6,1> update_eigen;
        update_eigen<<update[0],update[1],update[2],update[3],update[4],update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }
};

class EdgeProjection:public g2o::BaseUnaryEdge<2,Vector2d,VertexPose>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeProjection(const Vector3d &pos,const Matrix3d &K):_pos3d(pos),_k(K){}


    virtual void computeError() override {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Vector3d pos_pixel = _k*(T*_pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Vector3d pos_cam = T * _pos3d;
        double fx = _k(0,0);
        double cx = _k(0,2);
        double fy = _k(1,1);
        double cy = _k(1,2);
        double x = pos_cam[0];
        double y = pos_cam[1];
        double inv_z = 1.0/pos_cam[2];
        double inv_z2 = inv_z*inv_z;

        _jacobianOplusXi<<
        -fx*inv_z , 0 , fx*x*inv_z2 , fx*x*y*inv_z2 , -fx-fx*x*x*inv_z2 , fx*y*inv_z,
                0 , -fy*inv_z , fy*y*inv_z2 , fy+fy*y*inv_z2 ,- fy*x*y*inv_z2 , -fy*x*inv_z;

    }

    bool read(istream &is) override {
        return false;
    }

    bool write(ostream &os) const override {
        return false;
    }

private:
    Vector3d _pos3d;
    Matrix3d _k;
};

void bundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose) {
    cout<<"\nbundleAdjustmentG2O\n"<<endl;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton
            (g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(solver);

    VertexPose *vertexPose = new VertexPose();
    vertexPose->setId(0);
    vertexPose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertexPose);

    Matrix3d k_eigen;
    k_eigen<<K.at<double>(0,0),0,K.at<double>(2,2),
            0,K.at<double>(1,1),K.at<double>(1,2),
                    0,0,1;


    int index = 1;
    for(int i=0 ; i<points_2d.size();i++){
        auto p2d = points_2d[i];
        auto p3d = points_3d[3];
        EdgeProjection *edge = new EdgeProjection(p3d,k_eigen);
        edge->setId(index);
        edge->setVertex(0,vertexPose);
        edge->setMeasurement(p2d);
        edge->setInformation((Matrix2d::Identity()));
        optimizer.addEdge(edge);
        index++;
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout << "pose estimated by g2o =\n" << vertexPose->estimate().matrix() << endl;
    pose = vertexPose->estimate();
}

