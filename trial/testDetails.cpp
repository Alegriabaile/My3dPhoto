//
// Created by ale on 19-12-26.
//

#include <iostream>
#include <opencv2/opencv.hpp>
class A
{
public:
    union{
        size_t data[4];
        struct{
            size_t d1, d2, d3, d4;
        };
    };

    A(size_t data_[4]){
        for(size_t i = 0; i < 4; ++i)
            data[i] = data_[i];
    }
};

int main()
{
    //test the union and unnamed-struct-inside-union.
    size_t data_[4] = {4, 3, 2, 1};
    A a(data_);
    printf("data: %ld, %ld, %ld, %ld \n", a.d1, a.d2, a.d3, a.d4);


    cv::Mat image = cv::imread("trial/ghost rider.jpg", cv::IMREAD_ANYCOLOR || cv::IMREAD_ANYDEPTH);
    cv::imshow("image", image);

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::imshow("hsv", hsv);

    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), mask);
    cv::imshow("mask", mask);

    cv::Mat result = cv::Mat(image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    result.setTo(cv::Scalar(0, 0, 255));
    cv::bitwise_and(image, image, result, mask);
    cv::imshow("result", result);    
    
    cv::Mat testTranspose = image.clone();
    cv::imshow("transpose-original", testTranspose);
    cv::transpose(testTranspose, testTranspose);
    cv::imshow("transpose-transposed", testTranspose);
    
    
    cv::waitKey();

    return 0;
}