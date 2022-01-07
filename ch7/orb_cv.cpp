//
// Created by book on 2021/12/22.
//
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc,char **argv){
    cout<<"===========ch7:using opencv to inplement orb_feature extraction"<<endl;

    if(argc != 3){
        cerr<<"please input correct img_1 and img_2 path!!!"<<endl;
        return 1;
    }
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data !=nullptr);

    //初始化
    vector<KeyPoint> keypoint_1,keypoint_2;  //关键点
    Mat descriptors_1,descriptors_2;         // 描述子
    Ptr<FeatureDetector> detertor = ORB::create();  //角点探测器
    Ptr<DescriptorExtractor> descriptor = ORB::create();  // 描述子生成器
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");  //匹配器

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    //生成oriented FAST点
    detertor->detect(img_1,keypoint_1);
    detertor->detect(img_2,keypoint_2);

    //计算BRIEF
    descriptor->compute(img_1,keypoint_1,descriptors_1);
    descriptor->compute(img_2,keypoint_2,descriptors_2);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =  chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"extract orb 用时 :"<<time_used.count()<<" 秒"<<endl;

    //画出关键点
    Mat outimg1;
    drawKeypoints(img_1,keypoint_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("ORB feature",outimg1);

    //生成匹配信息
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();

    matcher->match(descriptors_1,descriptors_2,matches);

    t2 = chrono::steady_clock::now();
    time_used =  chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"match orb 用时 :"<<time_used.count()<<" 秒"<<endl;

    //对比：生成优化后的匹配信息
    vector<DMatch> good_matches;
    auto min_max = minmax_element(matches.begin(),matches.end(),
                   [](const DMatch m1,const DMatch m2){
        return (m1.distance<m2.distance);
    });
    double min_distance = min_max.first->distance;
    double max_distance = min_max.second->distance;
    printf("---max distance : %f\n",max_distance);
    printf("---min distance : %f\n",min_distance);

    for(auto m:matches){
        if(m.distance <= max(2*min_distance,30.0)){
            good_matches.push_back(m);
        }
    }

    //绘制匹配结果ss
    Mat img_match,img_goodmatch;
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,matches,img_match);
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,good_matches,img_goodmatch);
    imshow("match",img_match);
    imshow("good match",img_goodmatch);

    waitKey(0);
    return 0;
}
