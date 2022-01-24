//
// Created by book on 2022/1/7.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

string file_1 = "../LK1.png";  // first image
string file_2 = "../LK2.png";  // second image


void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false,
        bool has_initial_guess = false
);

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols - 1) x = img.cols - 2;
    if (y >= img.rows - 1) y = img.rows - 2;

    float xx = x - floor(x);
    float yy = y - floor(y);
    int x_a1 = std::min(img.cols - 1, int(x) + 1);
    int y_a1 = std::min(img.rows - 1, int(y) + 1);

    return (1 - xx) * (1 - yy) * img.at<uchar>(y, x)
           + xx * (1 - yy) * img.at<uchar>(y, x_a1)
           + (1 - xx) * yy * img.at<uchar>(y_a1, x)
           + xx * yy * img.at<uchar>(y_a1, x_a1);
}

int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    vector<KeyPoint> kp2_single;
    vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by gauss-newton: " << time_used.count() << endl;

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv: " << time_used.count() << endl;

    // plot the differences of those functions
    Mat img2_single;
    cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++) {
        if (success_single[i]) {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}

class OpticalFlowTracker{
public:
    OpticalFlowTracker(const Mat &img1_,const Mat &img2_,const vector<KeyPoint> &kp1_,
                       vector<KeyPoint> &kp2_,vector<bool> &success_,bool inverse_ = true,bool has_inital_ = false):
                       img1(img1_),img2(img2_),kp1(kp1_),kp2(kp2_),success(success_),inverse(inverse_),
                       has_inital(has_inital_){}

   void calculateOpticalFlow(const Range &range);

private:
    const Mat &img1;
    const Mat &img2;
    const vector<KeyPoint> &kp1;
    vector<KeyPoint> &kp2;
    vector<bool> &success;
    bool inverse = true;
    bool has_inital = false;

};

void OpticalFlowTracker::calculateOpticalFlow(const Range &range) {
    int half_patch_size = 4;
    int iterations = 10;
    for(std::size_t i=range.start;i<range.end;i++){
        auto kp = kp1[i];
        double dx=0,dy=0;
        if(has_inital){
            dx = kp2[i].pt.x - kp1[i].pt.x;
            dy = kp2[i].pt.y - kp1[i].pt.y;
        }

        double cost = 0,last_cost = 0;
        bool succ = true;
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d J;
        Eigen::Vector2d b = Eigen::Vector2d::Zero();

        for(int iter = 0;iter<iterations;iter++){
            if(inverse == false){
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            }else{
                b = Eigen::Vector2d::Zero();
            }

            cost = 0;
            for(int x=-half_patch_size ; x<half_patch_size ; x++)
                for(int y=-half_patch_size ; y<half_patch_size ; y++){
                    double error(GetPixelValue(img1,kp.pt.x+x,kp.pt.y+y)
                            - GetPixelValue(img2,kp.pt.x+x+dx,kp.pt.y+dy));
                    if(inverse == false){
                        J[0] = -0.5 * (GetPixelValue(img2,kp.pt.x+x+dx+1,kp.pt.y+dy+y) -
                                GetPixelValue(img2,kp.pt.x+dx+x-1,kp.pt.y+y+dy));
                        J[1] = -0.5 * (GetPixelValue(img2,kp.pt.x+x+dx,kp.pt.y+dy+y+1) -
                                       GetPixelValue(img2,kp.pt.x+dx+x,kp.pt.y+y+dy-1));
                    }else if(iter == 0){
                        J[0] = -0.5 * (GetPixelValue(img2,kp.pt.x+x+dx+1,kp.pt.y+dy+y) -
                                       GetPixelValue(img2,kp.pt.x+dx+x-1,kp.pt.y+y+dy));
                        J[1] = -0.5 * (GetPixelValue(img2,kp.pt.x+x+dx,kp.pt.y+dy+y+1) -
                                       GetPixelValue(img2,kp.pt.x+dx+x,kp.pt.y+y+dy-1));
                    }
                    b += error*J;
                    cost += error*error;
                    if(inverse == false || iter)
                        H += J*J.transpose();
                }

            Eigen::Vector2d update = H.ldlt().solve(b);
            if(std::isnan(update[0])){
                cout <<"update is nan"<<endl;
                succ = false;
                break;
            }

            if(iter > 0 && cost > last_cost){
                break;
            }

            dx += update[0];
            dy += update[1];
            last_cost = cost;
            succ = true;

            if(update.norm() < 1e-2){
                break;
            }
        }
       // cout<<"xxxx = "<<success.size()<<endl;
        success[i] = succ;
        kp2[i].pt = kp.pt + Point2f(dx,dy);
    }
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse,
        bool has_initial_guess){
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    cout<<"success = "<<success.size()<<endl;
    OpticalFlowTracker tracker(img1,img2,kp1,kp2,success,inverse,has_initial_guess);
    parallel_for_(Range(0,kp1.size()),
                  std::bind(&OpticalFlowTracker::calculateOpticalFlow,&tracker,placeholders::_1));

}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {
    int pyramids = 4;
    double pyramids_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    vector<Mat> pyr1, pyr2;
    for (int i = 0; i < pyramids; i++) {
        if (i == 0) {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        } else {
            Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyramids_scale * pyr1[i - 1].cols, pyramids_scale * pyr1[i - 1].rows));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyramids_scale * pyr2[i - 1].cols, pyramids_scale * pyr2[i - 1].rows));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);

        }
    }

    cout << "pyr1.size = " << pyr1.size() << endl;

    vector<KeyPoint> kp1_pyr, kp2_pyr;
    for (auto &kp:kp1) {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--) {
        success.clear();
        cout << "level = " << level << endl;
        cout << "pyr1[level] = " << pyr1[level].size << endl;

        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);
        if (level > 0) {
            for (auto &kp:kp1_pyr) {
                kp.pt /= pyramids_scale;
            }
            for (auto &kp:kp2_pyr) {
                kp.pt /= pyramids_scale;
            }
        }
    }

    for (auto &kp:kp2_pyr) {
        kp2.push_back(kp);
    }
}

