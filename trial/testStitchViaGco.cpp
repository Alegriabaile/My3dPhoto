//
// Created by ale on 19-12-24.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "GCoptimization.h"
#include "7PenaltiesGenerator.h"

#include "7LabelsFromErrors.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    int indices[13] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
//    std::vector<std::string> imgFileNames, dptFileNames;

    std::vector<cv::Mat> images, depths, errors;
    for(int i=0; i<13; ++i)
    {
        std::string imageFileName = "trial/input-data/" + to_string(indices[i]) + "_th_pano_Image.jpg";
        std::string depthFileName = "trial/input-data/" + to_string(indices[i]) + "_th_pano_Depth.png";
//        imgFileNames.push_back("trial/input-data/" + std::string(to_string(indices[i]) + "_th_pano_Image.jpg");
//        dptFileNames.push_back("trial/input-data/" + std::string(to_string(indices[i]) + "_th_pano_Depth.png");
        cv::Mat image = cv::imread(imageFileName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        cv::Mat depth = cv::imread(depthFileName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        cv::Mat depth_new;
        depth.convertTo(depth_new, CV_32FC1);
        images.push_back(image);
        depths.push_back(depth_new);
        errors.push_back(cv::Mat());

//        if(errors.back().empty())
//            printf("errors[%d].empty()...\n", i);

//        cv::imshow("image", image);
//        cv::imshow("depth", depth);
//        cv::waitKey();
    }

    cv::Mat lImage1;
    m3d::PenaltiesGenerator penaltiesGenerator(images, depths, errors, lImage1);

    size_t minHwMaxHw[4];
    getPanoBoundary(depths, minHwMaxHw);
    cv::Mat lImage2;
    getLabelsFromErrors(errors, minHwMaxHw, lImage2);

    cv::Mat pano_image1 = cv::Mat(lImage1.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat pano_image2 = cv::Mat(lImage1.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat pano_label_colored1 = cv::Mat(lImage1.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat pano_label_colored2 = cv::Mat(lImage1.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    cv::Mat pano_depth1 = cv::Mat(lImage1.size(), CV_32FC1, cv::Scalar(0, 0, 0));
    cv::Mat pano_depth2 = cv::Mat(lImage1.size(), CV_32FC1, cv::Scalar(0, 0, 0));

    size_t &minH = minHwMaxHw[0];
    size_t &minW = minHwMaxHw[1];
    size_t &maxH = minHwMaxHw[2];
    size_t &maxW = minHwMaxHw[3];
    size_t width = maxW - minW + 1;
    size_t height = maxH - minH + 1;
    std::vector<cv::Vec3b> vecColor = {cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0), cv::Vec3b(0, 0, 255), cv::Vec3b(255, 255, 0), cv::Vec3b(255, 0, 255),
                                       cv::Vec3b(0, 255, 255), cv::Vec3b(128, 255, 0), cv::Vec3b(255, 128, 0), cv::Vec3b(128, 0, 255), cv::Vec3b(255, 0, 128),
                                       cv::Vec3b(0, 128, 255), cv::Vec3b(0, 255, 128), cv::Vec3b(255, 255, 255)};
    for(size_t h = 0; h < height; ++h)
    {
        for(size_t w = 0; w < width; ++w)
        {
            size_t label = lImage1.at<uchar>(h + minH, w + minW);
            pano_image1.at<Vec3b>(h + minH, w + minW) = images[label].at<Vec3b>(h + minH, w + minW);
            pano_label_colored1.at<Vec3b>(h + minH, w + minW) = vecColor[(label+1)%vecColor.size()];
            pano_depth1.at<float>(h + minH, w + minW) = depths[label].at<float>(h + minH, w + minW);

            label = lImage2.at<uchar>(h + minH, w + minW);
            pano_image2.at<Vec3b>(h + minH, w + minW) = images[label].at<Vec3b>(h + minH, w + minW);
            pano_label_colored2.at<Vec3b>(h + minH, w + minW) = vecColor[(label+1)%vecColor.size()];
            pano_depth2.at<float>(h + minH, w + minW) = depths[label].at<float>(h + minH, w + minW);
        }
    }

    cv::Mat mask;
    cv::bitwise_not(pano_depth1>0, mask);
    pano_label_colored1.setTo(cv::Vec3b(0,0,0), mask);
    cv::bitwise_not(pano_depth2>0, mask);
    pano_label_colored2.setTo(cv::Vec3b(0,0,0), mask);

    cv::imwrite("trial/pano_image1.jpg", pano_image1);
    cv::imwrite("trial/pano_image2.jpg", pano_image2);
    cv::imwrite("trial/pano_label_colored1.jpg", pano_label_colored1);
    cv::imwrite("trial/pano_label_colored2.jpg", pano_label_colored2);



//    cv::detail::Blender blender;
    return 0;
}