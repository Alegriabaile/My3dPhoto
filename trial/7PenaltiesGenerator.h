//
// Created by ale on 19-12-3.
//

#ifndef MY3DPHOTO_7PENALTIESGENERATOR_H
#define MY3DPHOTO_7PENALTIESGENERATOR_H

#include <opencv2/opencv.hpp>

namespace m3d
{
    class PenaltiesGenerator
    {
    private:
        const std::vector<cv::Mat> &images, &depths;
        std::vector<cv::Mat> &errors;

        //generate the consensus penalty.
        void __GenerateConsensus(const cv::Mat &panoDepth1, const cv::Mat &panoDepth2, cv::Mat &panoError1, cv::Mat &panoError2);
        void GenerateRelativeConsensus(const size_t i, const size_t j);
        void GenerateConsensusPenalty();

        void GenerateSaturationPenalty();

        void GenerateBoundaryPenalty();

        void SetNonContentPenalty();

        void GeneratePenalties();


        //stitch with penalties.
        void GenerateLabels(cv::Mat &lImage);

//        void StitchPanoramasWithPenalties();

    public:
        PenaltiesGenerator(const std::vector<cv::Mat> &images_, const std::vector<cv::Mat> &depths_, std::vector<cv::Mat> &errors_);
        PenaltiesGenerator(const std::vector<cv::Mat> &images_, const std::vector<cv::Mat> &depths_, std::vector<cv::Mat> &errors_, cv::Mat &lImage_);
        virtual ~PenaltiesGenerator();
    };
}

#endif //MY3DPHOTO_7PENALTIESGENERATOR_H
