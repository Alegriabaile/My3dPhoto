//
// Created by ale on 19-12-27.
//

#include "7GcoStitchingSolver.h"

namespace m3d
{
    GcoStichingSolver::GcoStichingSolver(const std::vector<bool>& activatedFrames_, const std::vector<m3d::Frame> &frames_, m3d::Frame &result_)
    : activatedFrames(activatedFrames_), frames(frames_), result(result_)
    , dataTerms(NULL), smoothTerms(NULL), gc(NULL)
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
        if(gc != NULL)
            delete gc;
        gc = NULL;

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

        size_t &minH = result.minH;
        size_t &maxH = result.maxH;
        size_t &minW = result.minW;
        size_t &maxW = result.maxW;

        //todo... get the global/result minHwMaxHw from all frames.

        mHeight = result.maxH - result.minH + 1;
        mWidth = result.maxW - result.minW + 1;
        mNumLabels = mLabel2Index.size();

        size_t sz = mHeight * mWidth * mNumLabels;
        dataTerms = new int[sz];
        if(dataTerms == NULL)
        {
            printf("GcoStichingSolver::initialize(): initialize failed, dataTerms need larger memeries...");
            exit(-1);
        }
        memset(dataTerms, 500, mWidth, sz);
        //todo. set initial data terms.

        //initialize the
        sz = mNumLabels * mNumLabels;
        smoothTerms = new int[sz];
        if(dataTerms == NULL)
        {
            printf("GcoStichingSolver::initialize(): initialize failed, smoothTerms need larger memeries...");
            exit(-1);
        }
        for ( int l1 = 0; l1 < num_labels; l1++ )
            for (int l2 = 0; l2 < num_labels; l2++ )
                smoothTerms[l1+l2*num_labels] = (l1 == l2) ? 0:5;

    }

    void GcoStichingSolver::solve(int num_iters)
    {
        cv::Mat labelsImage = cv::Mat(mHeight, mWidth, CV_8UC1, cv::Scalar(0));
        try{
            gc = new GCoptimizationGridGraph(width,height,num_labels);
            if(gc == NULL)
            {
                printf("GcoStichingSolver::solve(): initialize gc failed...");
                exit(-1);
            }
            gc->setDataCost(dataTerm);
            gc->setSmoothCost(smoothTerm);

            printf("\nBefore optimization energy is %lld\n",gc->compute_energy());
            gc->expansion(num_iters);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
            printf("\nAfter optimization energy is %lld\n",gc->compute_energy());

            //read data from (GCoptimizationGridGraph* gc).
            for(size_t h = 0; h < mHeight; ++h)
                for(size_t w = 0; w < mWidth; ++w)
                    labelsImage.at<uchar>(h, w) = mLabel2Index[gc->whatLabel(h*width + w)];

            delete gc;
        }
        catch (GCException e){
            e.Report();
        }

        size_t &minH = result.minH;
        size_t &maxH = result.maxH;
        size_t &minW = result.minW;
        size_t &maxW = result.maxW;
        labelsImage.copyTo(result.pano_label(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
        //todo... update pano_image/depth/error/label_bgr...

    }


}