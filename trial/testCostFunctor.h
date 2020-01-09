//
// Created by ale on 20-1-6.
//

#ifndef MY3DPHOTO_TESTCOSTFUNCTOR_H
#define MY3DPHOTO_TESTCOSTFUNCTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "GCoptimization.h"
#include "testCostFunctor.h"

class ExtraData
{
public:
    const cv::Mat & srcImg;
    ExtraData( const cv::Mat &srcImg_)
            : srcImg(srcImg_){}
    ~ExtraData(){}
};
class ExtraSmoothData
{
public:
    const cv::Mat & srcImg;
    ExtraSmoothData( const cv::Mat &srcImg_)
            : srcImg(srcImg_){}
    ~ExtraSmoothData(){}
};

int dataCostFunction(int p, int l, void *data)
{
    ExtraData * extraData = (ExtraData*)data;
    const int cols = extraData->srcImg.cols;
    int h, w;
    h = p/cols;
    w = p%cols;
    int color = extraData->srcImg.at<uchar>(h, w);

    //
//    return 0;

    return std::abs(color/64-l)*10;
    if((color)/64 == l)
        return 0;
    return 10;
}
int smoothCostFunction(int p1, int p2, int l1, int l2, void *extraData)
{
    ExtraSmoothData * extraSmoothData = (ExtraSmoothData *)extraData;

    const int cols = extraSmoothData->srcImg.cols;
    int h1, w1, h2, w2;

    h1 = p1/cols; w1 = p1%cols;
    int color1 = extraSmoothData->srcImg.at<uchar>(h1, w1);

    h2 = p2/cols; w2 = p2%cols;
    int color2 = extraSmoothData->srcImg.at<uchar>(h2, w2);

    if( (std::abs(color1 - color2) < 30) && l1 != l2)
        return 10*std::abs(l1-l2);

//    if( (std::abs(color1 - color2) >= 50) && l1 == l2)
//        return 10;
    return 0;
}


#endif //MY3DPHOTO_TESTCOSTFUNCTOR_H
