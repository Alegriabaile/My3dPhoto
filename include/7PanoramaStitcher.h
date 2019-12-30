//
// Created by ale on 19-12-3.
//

#ifndef MY3DPHOTO_7PENALTIESGENERATOR_H
#define MY3DPHOTO_7PANORAMASTITCHER_H

#include "0m3d.h"

namespace m3d
{
    class PanoramaStitcher
    {
    private:
        const std::vector<bool> &activatedFrames;
        std::vector<m3d::Frame> &frames;

        m3d::Frame &result;

        //generate the consensus penalty.
        void GenerateRelativeConsensus(m3d::Frame &frame1, m3d::Frame &frame2);
        void GenerateConsensusPenalty();

        void GenerateSaturationPenalty();

        void GenerateBoundaryPenalty();

        void GeneratePenalties();


        //stitch with penalties.
        void GenerateLabelsNaive();
        void GenerateLabelsGco();

        void StitchPanoramasWithPenalties();

    public:
        PanoramaStitcher(const std::vector<bool> &activatedFrames, std::vector<m3d::Frame> &frames_, m3d::Frame &result_);
        virtual ~PanoramaStitcher();
    };
}

#endif //MY3DPHOTO_7PENALTIESGENERATOR_H
