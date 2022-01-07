//
// Created by book on 2021/12/26.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // use this if in OpenCV2

using namespace std;
using namespace cv;

/****************************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 * **************************************************/

void find_feature_matches(
        const Mat &img_1, const Mat &img_2,
        std::vector<KeyPoint> &keypoints_1,
        std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches);

void pose_estimation_2d2d(
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector<DMatch> matches,
        Mat &R, Mat &t);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv) {
    if(argc != 3){
        cerr<<"cannot get image 1,image 2!!!"<<endl;
        return 1;
    }
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "can not load image!!!");

    vector<KeyPoint> keypoints_1,keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    cout<<"一共上找到了"<<matches.size()<<"组匹配点"<<endl;

    //estimate
    Mat R,t;
    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);

    // 验证E = t^R*scale
    Mat t_x =
            (Mat_<double>(3,3) << 0 ,-t.at<double>(2,0) , t.at<double>(1,0),
                    t.at<double>(2,0),0,-t.at<double>(0,0),
                    -t.at<double>(1,0),t.at<double>(0,0),0);
    cout << "t^R = \n"<<t_x*R<<endl;

    // 验证对极约束  x2 * t^R * x1 = 0
    Mat k = (Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
    for(auto m:matches){
        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt,k);
        Mat y1 = (Mat_<double>(3,1)<<pt1.x,pt1.y,1);
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt,k);
        Mat y2 = (Mat_<double>(3,1)<<pt2.x,pt2.y,1);

        Mat d = y2.t() * t_x*R * y1;
        cout<<"x2^T * t^R * x1 = "<<d<<endl;
    }

    return 0;
}


void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, Mat &t) {
    Mat k = (Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);

    vector<Point2f> points_1,points_2;
    for(auto m:matches){
        points_1.push_back(keypoints_1[m.trainIdx].pt);
        points_2.push_back(keypoints_2[m.trainIdx].pt);
    }

    Mat fundamental_mat = findFundamentalMat(points_1,points_2,CV_FM_8POINT);
    cout << "fundamental_mat = \n"<<fundamental_mat<<endl<<endl;

    Point2d pricipal_point(325.1,249.9);
    double focal_length = 521;
    Mat essential_mat = findEssentialMat(points_1,points_2,focal_length,pricipal_point);
    cout << "essential_mat = \n"<<essential_mat<<endl<<endl;

    Mat homograph_mat = findHomography(points_1,points_2,RANSAC,3);
    cout << "homograph_mat = \n"<<homograph_mat<<endl<<endl;

    recoverPose(essential_mat,points_1,points_2,R,t,focal_length,pricipal_point);
    cout << "R = \n"<<R<<endl<<endl;
    cout << "t = \n"<<t.t()<<endl<<endl;

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
    //BFMatcher matcher ( NORM_HAMMING );
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

