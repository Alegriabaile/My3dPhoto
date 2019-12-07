//
// Created by ale on 19-12-5.
//

#include "8BackLayerContentGenerator.h"

namespace m3d
{

    BackLayerContentGenerator::BackLayerContentGenerator(m3d::Frame &result_)
//    : result(result_)
    {
        GenerateContent(result_);
    }

    BackLayerContentGenerator::~BackLayerContentGenerator()
    {

    }

    void BackLayerContentGenerator::_DEBUG_GenBoundary(const m3d::Frame &result_)
    {
        minH = m3d::Frame::PANO_H - 1;
        maxH = 0;
        minW = m3d::Frame::PANO_W - 1;
        maxW = 0;

        for(size_t h = 0; h < m3d::Frame::PANO_H; ++h)
        {
            for(size_t w = 0; w < m3d::Frame::PANO_W; ++w)
            {
                if(!(result_.pano_depth.at<float>(h,w) > 0))
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

        if(minH > maxH || minW > maxW)
        {
            std::cout<<"BackLayerContentGenerator::_DEBUG_GenBoundary: minH/W > maxH/W"<<std::endl;
            minH = maxH = minW = maxW = 0;
        }
    }

    void BackLayerContentGenerator::_DEBUG_ShowResults(const m3d::Frame &result_)
    {
        cv::imshow("front image", result_.pano_image(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
        cv::imshow("front depth", result_.pano_depth(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1))/10000.0f);
        cv::imshow("back depth", result_.pano_depth_b(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1))/10000.0f);
        cv::imshow("back mask", blocked(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
        cv::waitKey();
    }

    void BackLayerContentGenerator::MaxSpread(const size_t h, const size_t w, const cv::Mat& pano_depth_ref, const cv::Mat & blocked, cv::Mat& pano_depth_b)
    {
        const size_t H = pano_depth_ref.rows;
        const size_t W = pano_depth_ref.cols;

        if( h == 0 || h >= H || w == 0 || w >= W)
        {
            std::cout<<"BackLayerContentGenerator::MaxFilter(): h/w is out of ragne"<<std::endl;
            exit(-1);
        }

        float curVal = pano_depth_ref.at<float>(h, w);
        if(blocked.at<uchar>(h, w) || !(curVal > 0))
            return;

        size_t hs[4] = {h - 1, h, h + 1, h};
        size_t ws[4] = {w, w - 1, w, w + 1};
        float values[4];
        bool isBlocked[4];
        for(size_t i = 0; i < 4; ++ i)
        {
            values[i] = pano_depth_b.at<float>(hs[i], ws[i]);
            isBlocked[i] = blocked.at<uchar>(hs[i], ws[i]);
        }

        for(size_t i = 0; i < 4; ++i)
        {
            if(values[i] >= curVal || isBlocked[i])
                continue;
            pano_depth_b.at<float>(hs[i], ws[i]) = curVal;
        }

    }

    void BackLayerContentGenerator::GenerateBoundaryMask(m3d::Frame &result_)
    {
        const cv::Mat &pano_image_f = result_.pano_image;
        const cv::Mat &pano_depth_f = result_.pano_depth;

        //sharpen the boundary of objects.
        cv::medianBlur(pano_depth_f, pano_depth_f, 5);
        cv::medianBlur(pano_depth_f, pano_depth_f, 5);

        cv::Mat &pano_image_b = result_.pano_image_b;
        cv::Mat &pano_depth_b = result_.pano_depth_b;

        pano_image_b = cv::Mat(pano_image_f.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        pano_depth_b = cv::Mat(pano_depth_f.size(), CV_32FC1, cv::Scalar(0.0f));

        const size_t H = pano_image_f.rows;
        const size_t W = pano_image_f.cols;
        if(H == 0 || W == 0)
        {
            std::cout<<"BackLayerContentGenerator::GenerateBoundaryMask(m3d::Frame &result_): pano_image is empty???"<<std::endl;
            exit(-1);
        }

        //initialize.
        blocked = cv::Mat(H, W, CV_8UC1, cv::Scalar(0));
        float values[4];
        for(size_t h = 1; h < H - 1; ++h)
        {
            for(size_t w = 1; w < W - 1; ++w)
            {
                size_t hs[4] = {h - 1, h, h + 1, h};
                size_t ws[4] = {w, w - 1, w, w + 1};

                float curVal = pano_depth_f.at<float>(h, w);
                float maxVal = 0.0f;
                float minVal = 65500.0f;
                for(size_t i = 0; i < 4; ++i)
                {
                    values[i] = pano_depth_f.at<float>(hs[i], ws[i]);
                    maxVal = (maxVal > values[i]) ? maxVal : values[i];
                    minVal = (minVal < values[i]) ? minVal : values[i];
                }

                if( curVal > minVal)
                {
                    if( (curVal - minVal) > (0.02 * minVal))
                    {
                        pano_depth_b.at<float>(h, w) = curVal;
                        blocked.at<uchar>(h, w) = 255;
                    }
                }else if((maxVal - curVal) > (0.02 * curVal))
                    pano_depth_b.at<float>(h, w) = maxVal;
            }
        }

    }

    void BackLayerContentGenerator::IterativelyGenerateDepthContent(m3d::Frame &result_)
    {
        const cv::Mat &pano_depth_f = result_.pano_depth;
        cv::Mat &pano_depth_b = result_.pano_depth_b;

        const size_t H = pano_depth_f.rows;
        const size_t W = pano_depth_f.cols;
        if(H == 0 || W == 0)
        {
            std::cout<<"BackLayerContentGenerator::GenerateBoundaryMask(m3d::Frame &result_): pano_image is empty???"<<std::endl;
            exit(-1);
        }



        for(size_t i = 0; i < 30; ++i)
        {
            cv::Mat reference = pano_depth_b.clone();
            for(size_t h = 1; h < H - 1; ++h)
            {
                for(size_t w = 1; w < W - 1; ++w)
                {
                    MaxSpread(h, w, reference, blocked, pano_depth_b);
                }
            }

//            _DEBUG_ShowResults(result_);
        }

        pano_depth_b.setTo(0.0f, pano_depth_f > pano_depth_b);
    }

    void BackLayerContentGenerator::GenerateColorContent(m3d::Frame &result_)
    {
        cv::Mat mask = result_.pano_depth_b > 0;
        cv::inpaint(result_.pano_image, mask, result_.pano_image_b, 3, CV_INPAINT_NS);
    }

    void BackLayerContentGenerator::GenerateContent(m3d::Frame &result_)
    {
        std::cout<<"before _DEBUG_GenBoundary(result_)"<<std::endl;
        _DEBUG_GenBoundary(result_);

        std::cout<<"before GenerateBoundaryMask(result_)"<<std::endl;
        GenerateBoundaryMask(result_);

        std::cout<<"before IterativelyGenerateDepthContent(result_)"<<std::endl;
        IterativelyGenerateDepthContent(result_);

        std::cout<<"before GenerateColorContent(result_)"<<std::endl;
        GenerateColorContent(result_);
    }

}