//
// Created by ale on 19-12-3.
//
#include "1Logger.h"
#include "7PanoramaStitcher.h"

namespace m3d
{
    PanoramaStitcher::PanoramaStitcher(const std::vector<bool> &activatedFrames_, std::vector<m3d::Frame> &frames_, m3d::Frame &result_)
    : activatedFrames(activatedFrames_), frames(frames_), result(result_)
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
        GenerateLabelsNaive();
    }

    void PanoramaStitcher::GenerateRelativeConsensus(m3d::Frame &frame1, m3d::Frame &frame2)
    {
//        __GenerateConsensus(frame1.pano_depth, frame2.pano_depth, frame1.pano_error, frame2.pano_error);
        const cv::Mat &pano_depth1 = frame1.pano_depth;
        const cv::Mat &pano_depth2 = frame2.pano_depth;
        if(pano_depth1.empty() || pano_depth2.empty())
            return;

        cv::Mat &pano_error1 = frame1.pano_error;
        cv::Mat &pano_error2 = frame2.pano_error;
        if(pano_error1.empty())
            pano_error1 = cv::Mat(pano_depth1.size(), CV_32FC1, cv::Scalar(0.0f));
        if(pano_error2.empty())
            pano_error2 = cv::Mat(pano_depth2.size(), CV_32FC1, cv::Scalar(0.0f));

        const size_t H = pano_depth1.rows;
        const size_t W = pano_depth1.cols;
        const size_t H2 = pano_depth2.rows;
        const size_t W2 = pano_depth2.cols;

        const size_t minH1 = frame1.minH;
        const size_t minW1 = frame1.minW;
        const size_t minH2 = frame2.minH;
        const size_t minW2 = frame2.minW;

        for(size_t h = 0; h < H; ++h)
        {
            const int h2 = h + minH1 - minH2;
            if(h2 < 0 || h2 >= H2)
                continue;
            for(size_t w = 0; w < W; ++w)
            {
                const int w2 = w + minW1 - minW2;
                if(w2 < 0 || w2 >= W2)
                    continue;

                float d1 = pano_depth1.at<float>(h, w);
                float d2 = pano_depth2.at<float>(h2, w2);

                if(!(d1 > 0 && d2 > 0))
                    continue;

                float d1_div_d2 = d1/d2;
                if(d1_div_d2 < 0.9f || d1_div_d2 > 1.1f)
                    continue;

                pano_error1.at<float>(h, w) += 1.0f;
                pano_error2.at<float>(h2, w2) += 1.0f;
            }
        }
    }

    void PanoramaStitcher::GenerateConsensusPenalty()
    {
        if(frames.size() < 2)
        {
            m3d::LOG("PanoramaStitcher::GenerateConsensusPenalty()", "frames.size() < 2");
            exit(-1);
        }

        //generate the count of consensus depth.
        size_t sz = frames.size();
        for(size_t i = 0; i < sz - 1; ++i)
        {
            if(!activatedFrames[i])
                continue;

            for(size_t j = i + 1; j < sz; ++j)
            {
                if(!activatedFrames[j])
                    continue;

                GenerateRelativeConsensus(frames[i], frames[j]);
            }
        }

        //generate the consensus penalty.
        for(size_t i = 0; i < sz; ++i)
        {
            if(!activatedFrames[i])
                continue;

            cv::Mat &pano_error = frames[i].pano_error;
            const cv::Mat &pano_depth = frames[i].pano_depth;

            cv::Mat tmp_error = 1.0 - pano_error/5.0f;
            cv::Mat tmp_mask = (tmp_error > 0) & (pano_depth > 0);
            pano_error.setTo(0.0);

            tmp_error.copyTo(pano_error, tmp_mask);
        }

    }

    void PanoramaStitcher::GenerateSaturationPenalty()
    {
        size_t sz = frames.size();

        for(size_t i = 0; i < sz; ++i)
        {
            if(!activatedFrames[i])
                continue;

            m3d::Frame &frame = frames[i];
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

                    if(lab_current_saturation >=  0.98 * lab_max_saturation)
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
            if(!activatedFrames[i])
                continue;

            cv::Mat error = frames[i].pano_depth.clone();
            error.setTo(1.0f, error > 0);
            //(maxH - minH + 1, maxW - minW + 1, CV_32FC1, cv::Scalar(1.0f));
            size_t cols = error.cols;
            size_t rows = error.rows;
            float scale_min = 0.05f;
            float scale_max = 1.0f - scale_min*2.0f;
            cv::Rect roi(cols*scale_min, rows*scale_min, cols*scale_max, rows*scale_max);
            error(roi).setTo(0.0f);

            frames[i].pano_error += error;
        }
    }


    void PanoramaStitcher::GenerateLabelsNaive()
    {
        if(result.pano_label.empty())
        {
            result.pano_image = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC3, cv::Scalar(0,0,0));
            result.pano_depth = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(0.0));

            result.pano_error = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(500.0f));
            result.pano_label = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC1, cv::Scalar(255));
            result.pano_label_bgr = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC3, cv::Scalar(0,0,0));
        }
        result.minH = m3d::Frame::PANO_H - 1;
        result.maxH = 0;
        result.minW = m3d::Frame::PANO_W - 1;
        result.maxW = 0;
        size_t sz = frames.size();


        for(size_t i = 0; i < sz; ++ i) {
            //debug...
            double minV, maxV;
            cv::minMaxLoc(frames[i].pano_error, &minV, &maxV);
            std::cout<<"minV, maxV: "<<minV<<", "<<maxV<<std::endl;
            cv::Mat mask;
            cv::bitwise_not(frames[i].pano_depth > 0, mask);
            frames[i].pano_error.setTo(500.0f, mask);
        }

        cv::Mat pano_image_global = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat pano_depth_global = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(0.0));
        cv::Mat pano_error_global = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(500.0f));
        for(size_t i = 0; i < sz; ++ i)
        {
            if(!activatedFrames[i])
                continue;

            pano_image_global.setTo(cv::Scalar(0,0,0));
            pano_depth_global.setTo(0.0f);
            pano_error_global.setTo(500.0f);

            const size_t minH = frames[i].minH;
            const size_t maxH = frames[i].maxH;
            const size_t minW = frames[i].minW;
            const size_t maxW = frames[i].maxW;
            frames[i].pano_image.copyTo(pano_image_global(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
            frames[i].pano_depth.copyTo(pano_depth_global(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
            frames[i].pano_error.copyTo(pano_error_global(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));

            cv::Mat mask = (pano_error_global < result.pano_error) & (pano_depth_global > 0);
            result.pano_label.setTo(i, mask);

            cv::bitwise_and(pano_image_global, pano_image_global, result.pano_image, mask);
            cv::bitwise_and(pano_depth_global, pano_depth_global, result.pano_depth, mask);
            cv::bitwise_and(pano_error_global, pano_error_global, result.pano_error, mask);

            std::vector<cv::Vec3b> vecColor = {cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0), cv::Vec3b(0, 0, 255), cv::Vec3b(255, 255, 0), cv::Vec3b(255, 0, 255),
                                               cv::Vec3b(0, 255, 255), cv::Vec3b(128, 255, 0), cv::Vec3b(255, 128, 0), cv::Vec3b(128, 0, 255), cv::Vec3b(255, 0, 128),
                                               cv::Vec3b(0, 128, 255), cv::Vec3b(0, 255, 128), cv::Vec3b(255, 255, 255)};
            result.pano_label_bgr.setTo(vecColor[i%vecColor.size()], mask);

            result.minH = std::min(result.minH, minH);
            result.maxH = std::max(result.maxH, maxH);
            result.minW = std::min(result.minW, minW);
            result.maxW = std::max(result.maxW, maxW);
        }

        cv::Mat mask;
        cv::bitwise_not(result.pano_depth>0, mask);
        result.pano_label_bgr.setTo(cv::Vec3b(0,0,0), mask);
        result.pano_label.setTo(255, mask);
    }




}

