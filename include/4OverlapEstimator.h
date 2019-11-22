//
// Created by ale on 19-11-20.
//

#ifndef MY3DPHOTO_4OVERLAPESTIMATOR_H
#define MY3DPHOTO_4OVERLAPESTIMATOR_H

#include "opencv2/opencv.hpp"
#include "0m3d.h"

namespace m3d
{

    class OverlapEstimator
    {
    private:
        std::vector<m3d::Frame> &frames;
        m3d::Graph &graph;
        cv::Mat intrinsicK;

        void CalcInitialRt2d2d(Edge & edge, size_t index = 0);
        void CalcInitialRts2d2d();

    public:
        OverlapEstimator(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_);
        virtual ~OverlapEstimator();
    };

}



#endif //MY3DPHOTO_4OVERLAPESTIMATOR_H
