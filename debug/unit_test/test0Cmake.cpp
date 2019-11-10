//
// Created by ale on 19-11-6.
//

#include<iostream>

#include"0Frame.h"
#include "1FrameReader.h"

int main(int argc, char** argv)
{
    using namespace std;
    using namespace m3d;

    cv::Mat div = cv::Mat(20, 20, CV_32FC1, cv::Scalar(0));
    for(int i=0; i<20; ++i)
        div.at<float>(i, i) = 255;
    cv::Mat inv = 1/div;

    cout<<div<<endl;
    cout<<inv<<endl;


    return 0;
}