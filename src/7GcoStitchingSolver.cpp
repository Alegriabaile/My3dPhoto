//
// Created by ale on 19-12-27.
//

#include "7GcoStitchingSolver.h"
#include "GCoptimization.h"
namespace m3d
{
    GcoStichingSolver::GcoStichingSolver(const std::vector<bool>& activatedFrames_, const std::vector<m3d::Frame> &frames_, m3d::Frame &result_)
    : activatedFrames(activatedFrames_), frames(frames_), result(result_)
    , dataTerms(NULL), smoothTerms(NULL)
    {
        if(!initialize())
        {
            printf("GcoStichingSolver::GcoStichingSolver(): initialize failed, memery aquired...");
            exit(-1);
        };

        solve(20);
    }

    GcoStichingSolver::~GcoStichingSolver()
    {

        if(smoothTerms != NULL)
            delete[] smoothTerms;
        smoothTerms = NULL;

        if(dataTerms != NULL)
            delete[] dataTerms;
        dataTerms = NULL;
    }

    bool GcoStichingSolver::initialize()
    {
        mLabel2Index.clear();
        for(size_t i = 0; i < activatedFrames.size(); ++i)
            if(activatedFrames[i])
                mLabel2Index.push_back(i);
        if(mLabel2Index.empty())
        {
            printf("GcoStichingSolver::initialize(): mLabel2Index.empty().\n");
            exit(-1);
        }

        size_t &minH = result.minH;
        size_t &maxH = result.maxH;
        size_t &minW = result.minW;
        size_t &maxW = result.maxW;
        minH = frames[mLabel2Index[0]].maxH;
        maxH = 0;
        minW = frames[mLabel2Index[0]].maxW;
        maxW = 0;

        //todo... get the global/result minHwMaxHw from all frames.
        for(size_t i = 0; i < activatedFrames.size(); ++i)
            if(activatedFrames[i])
            {
                minH = std::min(minH, frames[i].minH);
                maxH = std::max(maxH, frames[i].maxH);
                minW = std::min(minW, frames[i].minW);
                maxW = std::max(maxW, frames[i].maxW);
            }


        mHeight = maxH - minH + 1;
        mWidth = maxW - minW + 1;
        mNumLabels = mLabel2Index.size();

        size_t sz = mHeight * mWidth * mNumLabels;
        dataTerms = new int[sz];
        if(dataTerms == NULL)
        {
            printf("GcoStichingSolver::initialize(): initialize failed, dataTerms need larger memeries...");
            exit(-1);
        }

        const int scl = 10;
        for(size_t i = 0; i < sz; ++i)
            dataTerms[i] = 500*scl;
//        memset(dataTerms, 500, sz*sizeof(int));
//        for(size_t i = 0; i < 20; ++i)
//            printf("%d", dataTerms[i]);
//        printf("\n");
//        for(size_t i = 0; i < 20; ++i)
//            printf("%d", dataTerms[sz - i - 1]);
//        printf("\n");

        //todo. set initial data terms.
        for(size_t i = 0; i < mLabel2Index.size(); ++i)
        {
            const size_t k = mLabel2Index[i];
            const size_t curMinH = frames[k].minH;
            const size_t curMaxH = frames[k].maxH;
            const size_t curMinW = frames[k].minW;
            const size_t curMaxW = frames[k].maxW;
            cv::Mat pano_error = frames[k].pano_error;
            cv::Mat mask;
            cv::bitwise_not(frames[k].pano_depth > 0, mask);
            pano_error.setTo(500*scl, mask);
            for(size_t h = curMinH - minH; h < curMaxH - minH + 1; ++h)
            {
                for(size_t w = curMinW - minW; w < curMaxW - minW + 1; ++w)
                {
                    float e = 1*pano_error.at<float>(h + minH - curMinH, w + minW - curMinW);
                    dataTerms[(h*mWidth + w)*mNumLabels + i] = e*float(scl);
//                    if(!(e > 0))
//                        printf("%f ", e);
                }
            }


        }

        //initialize the smoothness term.
        sz = mNumLabels * mNumLabels;
        smoothTerms = new int[sz];
        if(dataTerms == NULL)
        {
            printf("GcoStichingSolver::initialize(): initialize failed, smoothTerms need larger memeries...");
            exit(-1);
        }
        for ( int l1 = 0; l1 < mNumLabels; l1++ )
            for (int l2 = 0; l2 < mNumLabels; l2++ )
                smoothTerms[l1+l2*mNumLabels] = (l1 == l2) ? 0:5*scl;

        return true;
    }

    void GcoStichingSolver::solve(int num_iters)
    {
        cv::Mat labelsImage = cv::Mat(mHeight, mWidth, CV_8UC1, cv::Scalar(0));
        try{
            GCoptimizationGridGraph * gc = new GCoptimizationGridGraph(mWidth, mHeight, mNumLabels);

            gc->setDataCost(dataTerms);
            gc->setSmoothCost(smoothTerms);

            printf("\nBefore optimization energy is %lld\n",gc->compute_energy());
            gc->expansion(num_iters);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
            printf("\nAfter optimization energy is %lld\n",gc->compute_energy());

            //read data from (GCoptimizationGridGraph* gc).
            printf("GcoStichingSolver::solve(): reading labels from GraphCutOptimizer...\n");
            for(size_t h = 0; h < mHeight; ++h)
                for(size_t w = 0; w < mWidth; ++w)
                    labelsImage.at<uchar>(h, w) = mLabel2Index[gc->whatLabel(h*mWidth + w)];

            delete gc;
            delete[] smoothTerms;
            delete[] dataTerms;
            dataTerms = smoothTerms = NULL;
        }
        catch (GCException e){
            e.Report();
        }


        printf("GcoStichingSolver::solve(): start update from labels...\n");
        size_t &minH = result.minH;
        size_t &maxH = result.maxH;
        size_t &minW = result.minW;
        size_t &maxW = result.maxW;

        result.pano_label = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC1, cv::Scalar(255));
        result.pano_image = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC3, cv::Scalar(0,0,0));
        result.pano_depth = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(0.0));
        result.pano_error = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_32FC1, cv::Scalar(500.0f));
        result.pano_label_bgr = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC3, cv::Scalar(0,0,0));

        //todo... update pano_image/depth/error/label_bgr...
        //update the pano_label.
        labelsImage.copyTo(result.pano_label(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));

        //pre allocate memeries...
        const std::vector<cv::Vec3b> vecColor = {cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0), cv::Vec3b(0, 0, 255),
                                                 cv::Vec3b(255, 255, 0), cv::Vec3b(255, 0, 255), cv::Vec3b(0, 255, 255),
                                                 cv::Vec3b(128, 255, 0), cv::Vec3b(255, 128, 0), cv::Vec3b(128, 0, 255),
                                                 cv::Vec3b(255, 0, 128), cv::Vec3b(0, 128, 255), cv::Vec3b(0, 255, 128),
                                                 cv::Vec3b(255, 255, 255)};
        cv::Mat pano_image_global = result.pano_image.clone();
        cv::Mat pano_depth_global = result.pano_depth.clone();
        cv::Mat pano_error_global = result.pano_error.clone();
        cv::Mat mask = cv::Mat(m3d::Frame::PANO_H, m3d::Frame::PANO_W, CV_8UC1, cv::Scalar(0));
        for(size_t i = 0; i < mLabel2Index.size(); ++i)
        {
            //initialize...
            pano_image_global.setTo(cv::Scalar(0,0,0));
            pano_depth_global.setTo(0.0f);
            pano_error_global.setTo(500.0f);
            mask.setTo(0);
            //get the current mask.
            const size_t k = mLabel2Index[i];
            //update the current pano_image/depth/error.
            const size_t minH1 = frames[k].minH;
            const size_t maxH1 = frames[k].maxH;
            const size_t minW1 = frames[k].minW;
            const size_t maxW1 = frames[k].maxW;
            frames[k].pano_image.copyTo(pano_image_global(cv::Range(minH1, maxH1 + 1), cv::Range(minW1, maxW1 + 1)));
            frames[k].pano_depth.copyTo(pano_depth_global(cv::Range(minH1, maxH1 + 1), cv::Range(minW1, maxW1 + 1)));
            frames[k].pano_error.copyTo(pano_error_global(cv::Range(minH1, maxH1 + 1), cv::Range(minW1, maxW1 + 1)));

            mask = (result.pano_label == k) & (pano_depth_global > 0);

            cv::bitwise_and(pano_image_global, pano_image_global, result.pano_image, mask);
            cv::bitwise_and(pano_depth_global, pano_depth_global, result.pano_depth, mask);
            cv::bitwise_and(pano_error_global, pano_error_global, result.pano_error, mask);

            //visualizing the labels.
            result.pano_label_bgr.setTo(vecColor[i%vecColor.size()], mask);
        }

        cv::bitwise_not(result.pano_depth > 0, mask);
        result.pano_label.setTo(255, mask);

        printf("GcoStichingSolver::solve(): end update from labels...\n");
    }


}