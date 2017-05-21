//
//  main.cpp
//  homography
//
//  Created by 白田光 on 2017/05/18.
//  Copyright © 2017年 白田光. All rights reserved.
//

#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "lsd.h"


using namespace cv;
using namespace std;


int main(int argc, const char * argv[]) {
    Mat img = imread("/Users/Shirata/Desktop/data/PAS9.JPG",1);
    
    cv::resize(img, img, cv::Size(), 0.2, 0.2);
    
    Mat src_img = img.clone();
    
    cv::cvtColor(img, img, CV_BGR2GRAY);
    cv::GaussianBlur(img, img, cv::Size(7, 7), 1);
    cv::Mat element(5,5,CV_8U);
    cv::dilate(img, img, element, cv::Point(-1,-1), 1);
    
    //LSD用画像に変換
    double *dat = new double[img.rows * img.cols*2];
    for(int y = 0; y < img.rows; y++)
        for(int x = 0; x < img.cols; x++)
            dat[y * img.cols + x] = img.at<unsigned char>(y, x);
    
    int n_lines;
    double* lines;
    
    //LSD処理
    lines = lsd(&n_lines, dat, img.cols, img.rows);
    
    //しきい値の最大値と最小値をもってくる
    int max_NFA = 0;
    for(int i = 0; i < n_lines; i++)
        max_NFA = std::max(max_NFA, static_cast<int>(lines[i * 7 + 6]));
    
    vector<cv::Point> points;
    int flag = 0;
    
    while (points.size() < 4) {
        for(int i = 0; i < n_lines; i++){
            const double *line = &lines[i * 7];
            
            //        cout<< "NFA : " << nfa << endl;
            //        cout<< "line[5] : " << line[6] << endl;
            
            if(max_NFA < line[6]){
                cout << "check!" << endl;
                const cv::Point p1(line[0], line[1]);
                const cv::Point p2(line[2], line[3]);
                
                if(flag == 0){
                    points.push_back(p1);
                    points.push_back(p2);
                    flag = 1;
                }else{
                    if(p1 != points[0] && p2 != points[1]){
                        points.push_back(p1);
                        points.push_back(p2);
                    }
                }
                
                cout<< "P1 : " << p1 << endl;
                cout<< "P2 : " << p2<< endl;
                cout<< "points : " << points<< endl;
                
            }
        }
        max_NFA--;
    }
    
    cv::line(src_img, points[0], points[1], cv::Scalar(200,0,0), 2, CV_AA);
    cv::line(src_img, points[2], points[3], cv::Scalar(200,0,0), 2, CV_AA);
    
//    cv::Point2f src[4]; //変換前
//    cv::Point2f dst[4] = {Point(0,0),Point(20,0),Point(20,600),Point(0,600)}; // 変換先
//    
//    //線分が10°以下の時
//    double radian = atan2(abs(points[1].y - points[0].y), abs(points[1].x - points[0].x)) * (180/M_PI);
//    cout<< "radian : " << radian << endl;
//    if( radian < 90){
//        src[0] = points[1];
//        src[1] = points[2];
//        src[2] = points[3];
//        src[3] = points[0];
//    }else if( radian > 90){
//        src[0] = points[0];
//        src[1] = points[1];
//        src[2] = points[2];
//        src[3] = points[3];
//    }else{
//        cout << "other" << endl;
//    }
//    
//    //透視変換
//    cv::Mat perspective_matrix = cv::getPerspectiveTransform(src, dst);
//    cv::warpPerspective(src_img,img, perspective_matrix, img.size(), cv::INTER_LINEAR);

    //結果描画用画像
//    cv::cvtColor(img, img, CV_GRAY2BGR);

    
    imshow("image", src_img);
    waitKey(0);
    
    return 0;
   
}
