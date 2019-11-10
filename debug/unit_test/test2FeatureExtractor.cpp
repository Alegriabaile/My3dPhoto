//
// Created by ale on 19-11-8.
//

#include "2FeatureExtractor.h"


int main()
{
    std::string fileName1("data/01.jpg");
    std::string fileName2("data/02.jpg");

    std::vector<m3d::Frame> frames(2);

    frames[0].image = cv::imread(fileName1);
    frames[1].image = cv::imread(fileName2);

    m3d::FeatureExtractor::extractFeatures(frames);

    cv::Mat image1, image2;
    cv::drawKeypoints(frames[0].image, frames[0].keypoints, image1, cv::Scalar::all(-1), 4);
    cv::drawKeypoints(frames[0].image, frames[1].keypoints, image2, cv::Scalar::all(-1), 4);

    cv::imshow("image1", image1);
    cv::imshow("image2", image2);

    cv::waitKey();

    return 0;
}

