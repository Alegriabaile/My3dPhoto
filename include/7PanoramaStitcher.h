//
// Created by ale on 19-12-3.
//

#ifndef MY3DPHOTO_7PANORAMASTITCHER_H
#define MY3DPHOTO_7PANORAMASTITCHER_H

#include "0m3d.h"

namespace m3d
{
    class PanoramaStitcher
    {
    private:
        std::vector<m3d::Frame> &frames;
        m3d::Frame &result;

        //generate the consensus penalty.
        void __GenerateConsensus(const cv::Mat &panoDepth1, const cv::Mat &panoDepth2, cv::Mat &panoError1, cv::Mat &panoError2);
        void GenerateRelativeConsensus(m3d::Frame &frame1, m3d::Frame &frame2);
        void GenerateConsensusPenalty();

        void GenerateSaturationPenalty();

        void GenerateBoundaryPenalty();

        void GeneratePenalties();


        //stitch with penalties.
        void GenerateLabels();

        void StitchPanoramasWithPenalties();

    public:
        PanoramaStitcher(std::vector<m3d::Frame> &frames_, m3d::Frame &result_);
        virtual ~PanoramaStitcher();
    };
}

#endif //MY3DPHOTO_7PANORAMASTITCHER_H
