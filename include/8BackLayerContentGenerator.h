//
// Created by ale on 19-12-5.
//

#ifndef MY3DPHOTO_8BACKLAYERCONTENTGENERATOR_H
#define MY3DPHOTO_8BACKLAYERCONTENTGENERATOR_H

#include "0m3d.h"

namespace m3d
{
    class BackLayerContentGenerator
    {
    private:
//        m3d::Frame &result;
        cv::Mat blocked;
        size_t minH, maxH, minW, maxW;

        void _DEBUG_GenBoundary(const m3d::Frame &result_);
        void _DEBUG_ShowResults(const m3d::Frame &result_);

        void MaxSpread(size_t h, size_t w, const cv::Mat& pano_depth_ref, const cv::Mat & mask, cv::Mat& pano_depth_b);
        void GenerateBoundaryMask(m3d::Frame &result_);
        void IterativelyGenerateDepthContent(m3d::Frame &result_);
        void GenerateColorContent(m3d::Frame &result_);

        void GenerateContent(m3d::Frame &result_);

    public:
        BackLayerContentGenerator(m3d::Frame &result_);
        virtual ~BackLayerContentGenerator();
    };


}

#endif //MY3DPHOTO_8BACKLAYERCONTENTGENERATOR_H
