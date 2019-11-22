//
// Created by ale on 19-11-6.
//

#ifndef MY3DPHOTO_0FRAME_H
#define MY3DPHOTO_0FRAME_H

#include <opencv2/opencv.hpp>
#include "0CameraParameters.h"

namespace m3d
{
    class State
    {
    public:
        unsigned int id;
        State(unsigned int id_ = 0): id(id_){}
        ~State(){}
    };
    class Frame
    {
    public:
        std::string imageFileName;
        std::string depthFileName;
        std::string paramFileName;

        State state;
        static IntrinsicD intrinsicD;
        ExtrinsicD extrinsicD;

        cv::Mat image, depth, disparity;

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;

        cv::Mat pano_image, pano_depth, pano_error, pano_disparity;
        cv::Mat pano_image_b, pano_depth_b;
        cv::Mat pano_label;

        Frame() = default;

        virtual ~Frame() {};
    };


}

#endif //MY3DPHOTO_0FRAME_H
