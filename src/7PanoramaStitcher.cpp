//
// Created by ale on 19-12-3.
//
#include "1Logger.h"
#include "7PanoramaStitcher.h"

namespace m3d
{
    PanoramaStitcher::PanoramaStitcher(std::vector<m3d::Frame> &frames_, m3d::Frame &result_)
    : frames(frames_), result(result_)
    {
        GeneratePenalties();
        StitchPanoramasWithPenalties();
    }

    PanoramaStitcher::~PanoramaStitcher() {}

    void PanoramaStitcher::GeneratePenalties()
    {
        GenerateConsensusPenalty();
        GenerateSaturationPenalty();
        GenerateBoundaryPenalty();
    }

    void PanoramaStitcher::StitchPanoramasWithPenalties()
    {
        GenerateLabels();
    }

    void PanoramaStitcher::__GenerateConsensus(const cv::Mat &panoDepth1, const cv::Mat &panoDepth2,  cv::Mat &panoError1, cv::Mat &panoError2)
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

    void PanoramaStitcher::GenerateRelativeConsensus(m3d::Frame &frame1, m3d::Frame &frame2)
    {
        __GenerateConsensus(frame1.pano_depth, frame2.pano_depth, frame1.pano_error, frame2.pano_error);
    }

    void PanoramaStitcher::GenerateConsensusPenalty()
    {
        if(frames.empty())
        {
            m3d::LOG("PanoramaStitcher::GenerateConsensusPenalty()", "frames.empty()");
            exit(-1);
        }

        //generate the count of consensus depth.
        size_t sz = frames.size();
        for(size_t i = 0; i < sz - 1; ++i)
        {
            for(size_t j = 0; j < sz; ++j)
            {
                GenerateRelativeConsensus(frames[i], frames[j]);
            }
        }

        //generate the consensus penalty.
        for(size_t i = 0; i < sz - 1; ++i)
        {
            cv::Mat &pano_error = frames[i].pano_error;
            const cv::Mat &pano_depth = frames[i].pano_depth;

            if(pano_error.empty())
                continue;

            cv::Mat tmp_error = 1.0 - pano_error/3.0f;
            cv::Mat tmp_mask = (tmp_error > 0) & (pano_depth > 0);
            tmp_error.copyTo(pano_error, tmp_mask);
        }

    }

    void PanoramaStitcher::GenerateSaturationPenalty()
    {
        size_t sz = frames.size();

        for(size_t i = 0; i < sz; ++i)
        {
            m3d::Frame &frame = frames[i];
            if(frame.pano_depth.empty())
                continue;

            cv::Mat &pano_image = frame.pano_image;
            cv::Mat &pano_depth = frame.pano_depth;
            cv::Mat &pano_error = frame.pano_error;

            size_t H = frame.pano_image.rows;
            size_t W = frame.pano_image.cols;
            for(size_t h = 0; h < H; ++h)
            {
                for(size_t w = 0; w < W; ++w)
                {
                    if(!(pano_depth.at<float>(h, w) > 0))
                        continue;
                    cv::Vec3b vec3b = pano_image.at<cv::Vec3b>(h, w);
                    float lab_current_saturation = 0.4124 * vec3b[2] + 0.3576 * vec3b[1] + 0.1805 * vec3b[0];//bgr.
                    float lab_max_saturation = 0.4124 * 255 + 0.3576 * 255 + 0.1805 * 255;

                    if(lab_current_saturation < 0.8 * lab_max_saturation)
                        pano_error.at<float>(h, w) += 3;
                }
            }

        }

    }

    void PanoramaStitcher::GenerateBoundaryPenalty()
    {
        size_t sz = frames.size();

        for(size_t i = 0; i < sz; ++i)
        {
            size_t minH = m3d::Frame::PANO_H - 1;
            size_t maxH = 0;
            size_t minW = m3d::Frame::PANO_W - 1;
            size_t maxW = 0;

            for(size_t h = 0; h < m3d::Frame::PANO_H; ++h)
            {
                for(size_t w = 0; w < m3d::Frame::PANO_W; ++w)
                {
                    if(!(frames[i].pano_depth.at<float>(h,w) > 0))
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
            cv::Mat error = frames[i].pano_depth(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)).clone();
            error.setTo(1.0f, error > 0);
            //(maxH - minH + 1, maxW - minW + 1, CV_32FC1, cv::Scalar(1.0f));
            size_t cols = error.cols;
            size_t rows = error.rows;
            float scale_min = 0.05f;
            float scale_max = 1.0f - scale_min*2.0f;
            cv::Rect roi(cols*scale_min, rows*scale_min, cols*scale_max, rows*scale_max);
            error(roi).setTo(0.0f);

            frames[i].pano_error(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)) += error;
        }

    }


    void PanoramaStitcher::GenerateLabels()
    {
        if(result.pano_label.empty())
        {
            result.pano_image = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC3, cv::Scalar(0,0,0));
            result.pano_depth = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(0.0));
            result.pano_error = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(0.0));
            result.pano_label = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_16UC1, cv::Scalar(65500));
        }



    }

}

