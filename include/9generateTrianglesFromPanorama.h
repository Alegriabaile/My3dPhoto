//
// Created by ale on 19-12-7.
//

#ifndef MY3DPHOTO_9GENERATETRIANGLESFROMPANORAMA_H
#define MY3DPHOTO_9GENERATETRIANGLESFROMPANORAMA_H

#include <opencv2/opencv.hpp>
namespace m3d
{
    size_t generateTrianglesFromPanorama(const cv::Mat &pano_depth, std::vector<float> &out);
}
#endif //MY3DPHOTO_9GENERATETRIANGLESFROMPANORAMA_H
