//
// Created by ale on 19-12-5.
//

#include "0Filters.h"

namespace m3d
{
    class M3dFilter
    {
    public:
        virtual ~M3dFilter(){}

        virtual void operator()(const cv::Mat & src, cv::Mat & dst) = 0;
        virtual void operator()(const cv::Mat & src, cv::Mat & dst, size_t kernel) = 0;
    };

    class M3dMinFilter: public M3dFilter
    {
    public:
        virtual ~M3dMinFilter(){}

        void operator()(const cv::Mat & src_, cv::Mat & dst_)
        {
            this->operator()(src_, dst_, 3);
        }
        void operator()(const cv::Mat & src_, cv::Mat & dst_, size_t kernel)
        {
            const cv::Mat &src = src_;
            cv::Mat dst = src.clone();

            auto srcType = src.type();
            if(srcType != CV_32FC1)
            {
                std::cout<<"M3dMinFilter::operator(): src.type() not valid..."<<std::endl;
                exit(-1);
            }


            if(kernel%2 == 0)
                ++kernel;
            size_t H = src.rows;
            size_t W = src.cols;
            if(kernel > H || kernel > W)
            {
                std::cout<<"M3dMinFilter::operator(): kernel is largger than H/W..."<<std::endl;
                exit(-1);
            }

            size_t r = (kernel-1)/2;
            for(size_t h = r; h < H - r; ++h)
            {
                for(size_t w = r; w < W - r; ++w)
                {
                    size_t areas = kernel*kernel;
                    float &minVal = dst.at<float>(h, w);
                    for(size_t k = 0; k < areas; ++k)
                    {
                        size_t x = k % kernel;
                        size_t y = k / kernel;
                        float curVal = src.at<float>(h-r+y, w-r+x);
                        if(curVal < minVal)
                            minVal = curVal;
                    }
                }
            }

            dst_ = dst;
        }
    };

    class M3dMaxFilter: public M3dFilter
    {
    public:
        virtual ~M3dMaxFilter(){}

        void operator()(const cv::Mat & src_, cv::Mat & dst_)
        {
            this->operator()(src_, dst_, 3);
        }
        void operator()(const cv::Mat & src_, cv::Mat & dst_, size_t kernel)
        {
            const cv::Mat &src = src_;
            cv::Mat dst = src.clone();

            auto srcType = src.type();
            if(srcType != CV_32FC1)
            {
                std::cout<<"M3dMaxFilter::operator(): src.type() not valid..."<<std::endl;
                exit(-1);
            }


            if(kernel%2 == 0)
                ++kernel;
            size_t H = src.rows;
            size_t W = src.cols;
            if(kernel > H || kernel > W)
            {
                std::cout<<"M3dMaxFilter::operator(): kernel is largger than H/W..."<<std::endl;
                exit(-1);
            }

            size_t r = (kernel-1)/2;
            for(size_t h = r; h < H - r; ++h)
            {
                for(size_t w = r; w < W - r; ++w)
                {
                    size_t areas = kernel*kernel;
                    float &maxVal = dst.at<float>(h, w);
                    for(size_t k = 0; k < areas; ++k)
                    {
                        size_t x = k % kernel;
                        size_t y = k / kernel;
                        float curVal = src.at<float>(h-r+y, w-r+x);
                        if(curVal > maxVal)
                            maxVal = curVal;
                    }
                }
            }

            dst_ = dst;
        }
    };

    void applyM3dFilter(const cv::Mat & src, cv::Mat & dst, int filterType)
    {
        m3d::M3dFilter *m3dFilter =
                filterType == m3d::M3DFILTER_MIN ? (m3d::M3dFilter*)(new m3d::M3dMinFilter):
                filterType == m3d::M3DFILTER_MAX ? (m3d::M3dFilter*)(new m3d::M3dMaxFilter):
                0;

        if(!m3dFilter)
            std::cout<<"applyM3dFilter: unkown filter type."<<std::endl;

        (*m3dFilter)(src, dst);

        delete m3dFilter;
    }
}