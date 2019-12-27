//
// Created by ale on 19-12-3.
//
#include "7PenaltiesGenerator.h"

namespace m3d
{
    PenaltiesGenerator::PenaltiesGenerator(
            const std::vector<cv::Mat> &images_,
            const std::vector<cv::Mat> &depths_,
            std::vector<cv::Mat> &errors_)
            : images(images_), depths(depths_), errors(errors_)
    {
        GeneratePenalties();
    }
    PenaltiesGenerator::PenaltiesGenerator(
            const std::vector<cv::Mat> &images_,
            const std::vector<cv::Mat> &depths_,
            std::vector<cv::Mat> &errors_,
            cv::Mat &lImage_)
            : images(images_), depths(depths_), errors(errors_)
    {
        GeneratePenalties();
        GenerateLabels(lImage_);
    }
    PenaltiesGenerator::~PenaltiesGenerator() {}

    void PenaltiesGenerator::GeneratePenalties()
    {
        GenerateConsensusPenalty();
        GenerateSaturationPenalty();
        GenerateBoundaryPenalty();
        SetNonContentPenalty();
    }

    void PenaltiesGenerator::__GenerateConsensus(const cv::Mat &panoDepth1, const cv::Mat &panoDepth2,  cv::Mat &panoError1, cv::Mat &panoError2)
    {
        if(panoDepth1.empty() || panoDepth2.empty())
            return;

        if(panoError1.empty())
            panoError1 = cv::Mat(panoDepth1.size(), CV_32FC1, cv::Scalar(0.0f));
        if(panoError2.empty())
            panoError2 = cv::Mat(panoDepth2.size(), CV_32FC1, cv::Scalar(0.0f));


        size_t H = panoDepth1.rows;
        size_t W = panoDepth1.cols;
        for(size_t h = 0; h < H; ++h)
        {
            for(size_t w = 0; w < W; ++w)
            {
                float d1 = panoDepth1.at<float>(h, w);
                float d2 = panoDepth2.at<float>(h, w);

                if(!(d1 > 0 && d2 > 0))
                    continue;

                float d1_div_d2 = d1/d2;
                if(d1_div_d2 < 0.9f || d1_div_d2 > 1.1f)
                    continue;

                panoError1.at<float>(h, w) += 1.0f;
                panoError2.at<float>(h, w) += 1.0f;
            }
        }
    }

    void PenaltiesGenerator::GenerateRelativeConsensus(const size_t i, const size_t j)
    {
        __GenerateConsensus(depths[i], depths[j], errors[i], errors[j]);
    }

    void PenaltiesGenerator::GenerateConsensusPenalty()
    {
        if(images.size() < 2)
        {
            printf("PenaltiesGenerator::GenerateConsensusPenalty():    frames.size() < 2");
            exit(-1);
        }

        //generate the count of consensus depth.
        size_t sz = images.size();
        for(size_t i = 0; i < sz - 1; ++i)
        {
            if(images[i].empty())
                continue;

            for(size_t j = i+1; j < sz; ++j)
            {
                if(images[j].empty())
                    continue;

                GenerateRelativeConsensus(i, j);
            }
        }

        //generate the consensus penalty.
        for(size_t i = 0; i < sz; ++i)
        {
            if(images[i].empty())
                continue;

            cv::Mat &pano_error = errors[i];
            const cv::Mat &pano_depth = depths[i];

            cv::Mat tmp_error = 1.0 - pano_error/5.0f;
            cv::Mat tmp_mask = (tmp_error > 0) & (pano_depth > 0);
            pano_error.setTo(0.0);

            //debug...
//            double minV, maxV;
//            cv::minMaxLoc(errors[i], &minV, &maxV);
//            printf("GenerateConsensusPenalty: i=%ld: before, minV, maxV: %lf, %lf\n", i, minV, maxV);


            tmp_error.copyTo(pano_error, tmp_mask);

            //debug...
//            cv::minMaxLoc(errors[i], &minV, &maxV);
//            printf("GenerateConsensusPenalty: i=%ld: after, minV, maxV: %lf, %lf\n ", i, minV, maxV);
//            cv::Mat tmperror;
//            cv::resize(errors[i], tmperror, errors[i].size()/2);
//            cv::imshow("errors[i]", tmperror);
//            cv::waitKey();
        }

    }

    void PenaltiesGenerator::GenerateSaturationPenalty()
    {
        size_t sz = images.size();

        for(size_t i = 0; i < sz; ++i)
        {
            if(images[i].empty())
                continue;

            const cv::Mat &pano_image = images[i];
            const cv::Mat &pano_depth = depths[i];
            cv::Mat &pano_error = errors[i];


            //debug...
//            double minV, maxV;
//            cv::minMaxLoc(errors[i], &minV, &maxV);
//            printf("GenerateSaturationPenalty: i=%ld: before, minV, maxV: %lf, %lf\n", i, minV, maxV);

            size_t H = pano_image.rows;
            size_t W = pano_image.cols;
            for(size_t h = 0; h < H; ++h)
            {
                for(size_t w = 0; w < W; ++w)
                {
                    if(!(pano_depth.at<float>(h, w) > 0))
                        continue;
                    cv::Vec3b vec3b = pano_image.at<cv::Vec3b>(h, w);
                    float lab_current_saturation = 0.4124 * vec3b[2] + 0.3576 * vec3b[1] + 0.1805 * vec3b[0];//bgr.
                    float lab_max_saturation = 0.4124 * 255 + 0.3576 * 255 + 0.1805 * 255;

                    //τsaturated = 0.98
                    if(lab_current_saturation >=  0.98 * lab_max_saturation)
                        pano_error.at<float>(h, w) += 3;
                }
            }

            //debug...
//            cv::minMaxLoc(errors[i], &minV, &maxV);
//            printf("GenerateSaturationPenalty: i=%ld: after, minV, maxV: %lf, %lf\n ", i, minV, maxV);
//            cv::Mat tmperror, tmpcolor;
//            cv::resize(errors[i], tmperror, errors[i].size()/2);
//            cv::imshow("errors[i]", tmperror/4.0f);
//            cv::resize(images[i], tmpcolor, images[i].size()/2);
//            cv::imshow("images[i]", tmpcolor);
//            cv::waitKey();

        }

    }

    void PenaltiesGenerator::GenerateBoundaryPenalty()
    {
        size_t sz = images.size();

        for(size_t i = 0; i < sz; ++i)
        {
            if(images[i].empty())
                continue;

            size_t PANOH = images[i].rows;
            size_t PANOW = images[i].cols;
            size_t minH = PANOH - 1;
            size_t maxH = 0;
            size_t minW = PANOW - 1;
            size_t maxW = 0;

            for(size_t h = 0; h < PANOH; ++h)
            {
                for(size_t w = 0; w < PANOW; ++w)
                {
                    if(!(depths[i].at<float>(h,w) > 0))
                        continue;

                    if(minH > h)
                        minH = h;
                    if(maxH < h)
                        maxH = h;

                    if(minW > w)
                        minW = w;
                    if(maxW < w)
                        maxW = w;
                }
            }
            cv::Mat error = depths[i](cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)).clone();
            error.setTo(1.0f, error > 0);
            //(maxH - minH + 1, maxW - minW + 1, CV_32FC1, cv::Scalar(1.0f));
            size_t cols = error.cols;
            size_t rows = error.rows;
            float scale_min = 0.05f;
            float scale_max = 1.0f - scale_min*2.0f;
            cv::Rect roi(cols*scale_min, rows*scale_min, cols*scale_max, rows*scale_max);
            error(roi).setTo(0.0f);

            //debug...
//            cv::imshow("error..", error);
//            double minV, maxV;
//            cv::minMaxLoc(errors[i], &minV, &maxV);
//            printf("GenerateBoundaryPenalty: i=%ld: before, minV, maxV: %lf, %lf\n", i, minV, maxV);

            errors[i](cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)) += error;

            //debug...
//            cv::minMaxLoc(errors[i], &minV, &maxV);
//            printf("GenerateBoundaryPenalty: i=%ld: after, minV, maxV: %lf, %lf\n ", i, minV, maxV);
//            cv::Mat tmperror, tmpcolor;
//            cv::resize(errors[i], tmperror, errors[i].size()/2);
//            cv::imshow("errors[i], after", tmperror/5.0f);
//            cv::waitKey();
        }

    }

    void PenaltiesGenerator::SetNonContentPenalty()
    {
        size_t sz = depths.size();
        for(size_t i = 0; i < sz; ++i)
        {
            //debug...
            double minV, maxV;
            cv::minMaxLoc(errors[i], &minV, &maxV);
            std::cout<<"minV, maxV: "<<minV<<", "<<maxV<<std::endl;

            cv::Mat mask;
            cv::bitwise_not(depths[i] > 0, mask);
            errors[i].setTo(500.0f, mask);

            //debug...
//            cv::Mat tmperror;
//            cv::resize(errors[i], tmperror, errors[i].size()/2);
//            cv::imshow("errors[i]", tmperror/20.0f);
//            cv::waitKey();
        }
    }

    void PenaltiesGenerator::GenerateLabels(cv::Mat &lImage)
    {
        if(lImage.empty())
        {
            lImage = cv::Mat(images[0].size(), CV_8UC1, cv::Scalar(0));
        }
        cv::Mat error = cv::Mat(images[0].size(), CV_32FC1, cv::Scalar(500.0f));
        size_t sz = images.size();
        for(size_t i = 0; i < sz; ++i)
        {
            if(images[i].empty())
                continue;

            cv::Mat mask = (errors[i] < error) & (depths[i] > 0);
            lImage.setTo(i, mask);
        }
    }
}

