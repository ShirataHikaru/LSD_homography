//
//  main.cpp
//  homography
//
//  Created by 白田光 on 2017/05/18.
//  Copyright © 2017年 白田光. All rights reserved.
//

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "lsd.h"
#include<iostream>
#include<fstream>

#define ROI_WIDTH 60
#define ROI_HEIGHT 1800

using namespace cv;
using namespace std;

string file_name = "PAS15";
string root_file_path = "/Users/Shirata/Desktop/data/";
string csv_file_path = "/Users/Shirata/Desktop/data/csv/";
string roi_file_path = "/Users/Shirata/Desktop/data/roi/";


//string image_path = "/Users/Shirata/Desktop/data/角度チェック/-15°/-15°.001.jpeg";

void CreateCsv(cv::Mat img);
vector<cv::Point> CustomLSD(int max_nfa,int n_lines,double* lines);

int main(int argc, const char * argv[]) {
    
    Mat img = imread(root_file_path+file_name+".JPG",1);
    
//    Mat img = imread(image_path,1);
    
    Mat src_img = img.clone();
    
    cv::resize(img, img, cv::Size(), 0.2, 0.2);
    
    Mat dst_img = img.clone();
    
    vector<cv::Mat> channels;
    cv::split(img, channels);
    
//    cv::cvtColor(img, img, CV_BGR2GRAY);
    
    cv::GaussianBlur(channels[2], img, cv::Size(7, 7), 1);
    
    
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
    
    //CustomLSDで得られた一番長い2辺の４座標を格納する
    vector<cv::Point> vertical_points = CustomLSD(max_NFA, n_lines, lines);
    
    //PASの長辺２本
    cv::line(dst_img, vertical_points[0], vertical_points[1], cv::Scalar(200,0,0), 2, CV_AA);
    cv::line(dst_img, vertical_points[2], vertical_points[3], cv::Scalar(200,0,0), 2, CV_AA);

//    //上辺，下辺の線分
//    cv::line(dst_img, get<0>(upperT), get<1>(upperT), cv::Scalar(0,0,200), 5, CV_AA);
//    cv::line(dst_img, get<0>(lowerT), get<1>(lowerT), cv::Scalar(0,200,0), 2, CV_AA);
    
    cv::Point2f src[4]; //変換前
    cv::Point2f dst[4] = {Point(0,0),Point(ROI_WIDTH,0),Point(ROI_WIDTH,ROI_HEIGHT),Point(0,ROI_HEIGHT)}; // 変換先
    
    //線分が90°以下の時
    double radian = 90 - atan2(abs(vertical_points[1].y - vertical_points[0].y), abs(vertical_points[1].x - vertical_points[0].x)) * (180/M_PI);
    cout<< "radian : " << radian << endl;
    if( radian < 90){
        src[0] = vertical_points[1]*5;
        src[1] = vertical_points[2]*5;
        src[2] = vertical_points[3]*5;
        src[3] = vertical_points[0]*5;
        
        cout << "less than 90°" << endl;
    }else if( radian > 90){
        src[0] = vertical_points[0]*5;
        src[1] = vertical_points[1]*5;
        src[2] = vertical_points[2]*5;
        src[3] = vertical_points[3]*5;
        cout << "larger than 90°" << endl;
    }else{
        cout << "other" << endl;
    }

    //透視変換
    cv::Mat perspective_matrix = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(src_img, img, perspective_matrix, src_img.size(), cv::INTER_LINEAR);

//    結果描画用画像
//    cv::cvtColor(img, img, CV_GRAY2BGR);
    
    namedWindow("img",CV_WINDOW_KEEPRATIO);
    imshow("img", img);
    
    namedWindow("dst_img",CV_WINDOW_KEEPRATIO);
    imshow("dst_img", dst_img);
    
    Rect roi(0,0,ROI_WIDTH,ROI_HEIGHT);
    
//    cvtColor(img, img, CV_BGR2HSV);
//    split(img,channels);
    
    Mat image_roi = img(roi);
    
//    CreateCsv(image_roi);
    
//    imwrite(roi_file_path+file_name+"_roi.jpg", image_roi);
    imshow("image_roi", image_roi);
    
    waitKey(0);
    
    return 0;
   
}


/*! @brief 画像の画素の値をcsvに書き出す
 @param[in] img 1チャンネルの画像
 */
void CreateCsv(cv::Mat img){
    ofstream ofs(csv_file_path+file_name+".csv"); //ファイル出力ストリーム
    
    //header
    for(int i=0; i < img.cols; i++){
        ofs << "Col" << i << "," <<flush;
    }
    ofs<< "\n" <<flush;
    
    //body
    for(int i = 0;i < img.rows;i++){
        for (int j = 0; j < img.cols; j++) {
            ofs << int(img.at<unsigned char>(i,j)) << "," <<flush;
        }
        ofs<< "\n" <<flush;
    }
}


/*! @brief Line Segment Detectorを使って一番長い直線２本の端点座標を返す
 @param[in] max_nfa max_NFA
 @param[in] n_lines n_lines
 @param[in] lines lines
 @return vector<cv::Point>に格納された直線座長を返す
 */
vector<cv::Point> CustomLSD(int max_nfa,int n_lines, double* lines){
    int flag = 0;
    
    //水平な直線と，垂直な直線を格納するためのvector
    vector<cv::Point> vertical_points;
    vector<cv::Point> horizontal_points;
    
    //max_NFAが０になるまで減らす
    while (max_nfa != 0) {
        for(int i = 0; i < n_lines; i++){
            const double *line = &lines[i * 7];
            
            if(max_nfa < line[6]){
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
        max_nfa--;
    }

    return vertical_points;
}


