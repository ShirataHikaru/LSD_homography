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
    Mat img = imread("/Users/Shirata/Desktop/data/PAS16.JPG",1);
    
    cv::resize(img, img, cv::Size(), 0.2, 0.2);
    
    Mat src_img = img.clone();
    
    vector<cv::Mat> channels;
    cv::split(img, channels);
    
//    cv::cvtColor(img, img, CV_BGR2GRAY);
    
    cv::GaussianBlur(channels[2], img, cv::Size(7, 7), 1);
    
//    cv::Mat element(7,7,CV_8U);
//    cv::dilate(img, img, element, cv::Point(-1,-1), 1);
//    cv::erode(img, img, element,cv::Point(-1,-1), 1);
    
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
        max_NFA = max(max_NFA, static_cast<int>(lines[i * 7 + 6]));
    
    //水平な直線と，垂直な直線を格納するためのvector
    vector<cv::Point> vertical_points;
    vector<cv::Point> horizontal_points;
    
    int flag = 0;
    
    while (max_NFA != 0) {
        for(int i = 0; i < n_lines; i++){
            const double *line = &lines[i * 7];
            
            if(max_NFA < line[6]){
                const cv::Point p1(line[0], line[1]);
                const cv::Point p2(line[2], line[3]);
                
                //線分の角度を求める
                double radian = atan2( abs(p2.y - p1.y), abs(p2.x - p1.x)) * (180 / M_PI);
                //検出された直線が10°以下の時(水平に近しい)，vectorに格納する
                if( radian < 10 ){
                    horizontal_points.push_back(p1);
                    horizontal_points.push_back(p2);
                }
                
                //検出された直線が画像の際だった時vectorに格納されるのを防ぐ
                if(p1.x < 20 || p2.x < 20 || p1.y < 20 || p2.y < 20){
                    break;
                }
                
                //とりあえず一回vectorに格納
                if(flag == 0){
                    vertical_points.push_back(p1);
                    vertical_points.push_back(p2);
                    cout << "Check" << endl;
                    flag++;
                //最初に格納した値と比較して異なればvectorにプッシュする
                }else if(flag == 1){
                    if(p1 != vertical_points[0] && p2 != vertical_points[1]){
                        vertical_points.push_back(p1);
                        vertical_points.push_back(p2);
                        //vectorに４つ以上値が入った時フラグをインクリメント
                        if(vertical_points.size() == 4) flag++;
                    }
                }
            }
        }
        max_NFA--;
    }
    
    //上辺と下辺の座標をタプルで確保する
    tuple<cv::Point,cv::Point> upperT, lowerT;
    upperT = make_tuple(horizontal_points[0],horizontal_points[1]);
    lowerT = make_tuple(horizontal_points[0],horizontal_points[1]);
    
    for (int i= 0; i < horizontal_points.size(); i += 2) {
        //上辺の線分を求める
        if(min(get<0>(upperT).y,get<1>(upperT).y) > min(horizontal_points[i].y,horizontal_points[i+1].y)){
            upperT = make_tuple(horizontal_points[i], horizontal_points[i+1]);
        }
        //下辺の線分を求める
        if(max(get<0>(lowerT).y,get<1>(lowerT).y) < max(horizontal_points[i].y,horizontal_points[i+1].y)){
            lowerT = make_tuple(horizontal_points[i], horizontal_points[i+1]);
        }
    }
    
    //PASの長辺２本
    cv::line(src_img, vertical_points[0], vertical_points[1], cv::Scalar(200,0,0), 2, CV_AA);
    cv::line(src_img, vertical_points[2], vertical_points[3], cv::Scalar(200,0,0), 2, CV_AA);
    
    //上辺，下辺の線分
    cv::line(src_img, get<0>(upperT), get<1>(upperT), cv::Scalar(0,0,200), 5, CV_AA);
    cv::line(src_img, get<0>(lowerT), get<1>(lowerT), cv::Scalar(0,200,0), 2, CV_AA);
    
    
    cv::Point2f src[4]; //変換前
    cv::Point2f dst[4] = {Point(0,0),Point(20,0),Point(20,600),Point(0,600)}; // 変換先
    
    //線分が10°以下の時
    double radian = atan2(abs(vertical_points[1].y - vertical_points[0].y), abs(vertical_points[1].x - vertical_points[0].x)) * (180/M_PI);
    cout<< "radian : " << radian << endl;
    if( radian < 90){
        src[0] = vertical_points[1];
        src[1] = vertical_points[2];
        src[2] = vertical_points[3];
        src[3] = vertical_points[0];
        
        cout << "less than 90°" << endl;
    }else if( radian > 90){
        src[0] = vertical_points[0];
        src[1] = vertical_points[1];
        src[2] = vertical_points[2];
        src[3] = vertical_points[3];
        cout << "larger than 90°" << endl;
    }else{
        cout << "other" << endl;
    }
    
    //透視変換
    cv::Mat perspective_matrix = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(src_img,img, perspective_matrix, img.size(), cv::INTER_LINEAR);

//    結果描画用画像
//    cv::cvtColor(img, img, CV_GRAY2BGR);

    imshow("img", img);
    imshow("src_img", src_img);
    
    waitKey(0);
    
    return 0;
   
}
