//
// Created by book on 2021/12/6.
//
#include <iostream>
#include <chrono>   //计时所用

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

/*
 * 基于Opencv对图像的一些基本操作
 * */
int main(int argc, char* argv[]){

    cv::Mat image;
    image = cv::imread(argv[1]);

    if(image.data == nullptr){
        cerr<<" 您输入的路径："<<argv[1]<<" 文件不存在!"<<endl;
        return 0;
    }

    cout<<"image  宽col = "<<image.cols<<"   ,高raw = "<<image.rows<<"   通道数channels = "<<image.channels()<<endl;
    cv::imshow("ubuntu raw image",image);
    cv::waitKey(0);

    cout<<"image.type() = "<<image.type()<<endl;
    if(image.type() != CV_8UC1 && image.type()!= CV_8UC3){
        cout <<"图像类型不符合要求，请输入一张彩色图或灰度图！"<<endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    //遍历图像
    for(size_t y=0 ; y < image.rows ; y++){
        unsigned char *row_ptr = image.ptr<unsigned char>(y);    //.ptrs是内联函数,传入的模板参数是step

        for(size_t x=0 ; x < image.cols ; x++ ){
            unsigned char *data_ptr = (unsigned char*)(row_ptr+ x*image.channels());

            //输出该像素的每个通道
            for(int c=0;c<image.channels();c++){
                unsigned char data = data_ptr[c];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"遍历图像用时 :"<<time_used.count()<<" 秒"<<endl;

///下面验证浅拷贝、深拷贝/////////////////////////////////////////////////////////////////////////////////////////////////////////

    //浅拷贝，修改image会导致image2变换
    cv::Mat image2 = image;
    image2(cv::Rect(0,0,200,200)).setTo(cv::Scalar(0,255,0));
    cv::imshow("image",image);
    cv::waitKey(0);

    //深拷贝
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0,0,200,200)).setTo(cv::Scalar(255,0,0));
    cv::imshow("image_clone",image_clone);
    cv::imshow("image",image);
    cv::waitKey(0);

    cv::destroyAllWindows();

    return 0;
}