//
// Created by ale on 19-12-5.
//

#ifndef MY3DPHOTO_0FILTERS_H
#define MY3DPHOTO_0FILTERS_H

#include <opencv2/opencv.hpp>

namespace m3d
{
    enum M3dFilterTypes
    {
        M3DFILTER_MIN = 0,
        M3DFILTER_MAX = 1
    };

    void applyM3dFilter(const cv::Mat & src, cv::Mat & dst, int filterType);
}

#endif //MY3DPHOTO_0FILTERS_H
