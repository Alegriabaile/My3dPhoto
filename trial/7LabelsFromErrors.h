//
// Created by ale on 19-12-25.
//

#ifndef MY3DPHOTO_7LABELSFROMERRORS_H
#define MY3DPHOTO_7LABELSFROMERRORS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "GCoptimization.h"

void getPanoBoundary(const std::vector<cv::Mat> &depths, size_t minHwMaxHw[4])
{
    printf("getPanoBoundary(): start...\n");
    if(minHwMaxHw == NULL)
    {
        std::cout<<"getPanoBoundary(): minHwMaxHw is empty...\n";
        exit(-1);
    }

    if(depths.empty())
    {
        std::cout<<"getPanoBoundary(): depths.empty()...\n";
        exit(-2);
    }

    size_t &minH = minHwMaxHw[0];
    size_t &minW = minHwMaxHw[1];
    size_t &maxH = minHwMaxHw[2];
    size_t &maxW = minHwMaxHw[3];

    minH = depths[0].rows - 1;
    minW = depths[0].cols - 1;
    maxH = maxW = 0;

    size_t sz = depths.size();

    for(size_t i = 0; i < sz; ++i)
    {
        if(depths[i].empty())
            continue;

        for(size_t h = 0; h < depths[i].rows; ++h)
        {
            for(size_t w = 0; w < depths[i].cols; ++w)
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
    }

    printf("getPanoBoundary(): end...\n");
}

void getLabelsFromErrors(const std::vector<cv::Mat> &errors, const size_t *const minHwMaxHw, cv::Mat &lImage)
{
    printf("getLabelsFromErrors(): start...\n");
    size_t num_labels = errors.size();
    if(num_labels == 0)
    {
        std::cout<<"getLabelsFromErrors(): num_labels == 0"<<std::endl;
        exit(-1);
    }

    if(minHwMaxHw == NULL)
    {
        std::cout<<"getLabelsFromErrors(): minHwMaxHw == NULL"<<std::endl;
        exit(-2);
    }

    size_t rows = errors[0].rows;
    size_t cols = errors[0].cols;
    lImage = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));

    size_t minH = minHwMaxHw[0];
    size_t minW = minHwMaxHw[1];
    size_t maxH = minHwMaxHw[2];
    size_t maxW = minHwMaxHw[3];

    size_t width = maxW - minW + 1;
    size_t height = maxH - minH + 1;
    size_t num_pixels = width*height;
    int *dataTerm = new int[num_pixels*num_labels];
    for(size_t h = 0; h < height; ++h)
        for(size_t w = 0; w < width; ++w)
            for(size_t l = 0; l < num_labels; ++l)
            {
                float e = errors[l].at<float>(h + minH, w + minW);
                dataTerm[(h*width + w)*num_labels + l] = e;
            }


    int *smoothTerm = new int[num_labels*num_labels];
    for ( int l1 = 0; l1 < num_labels; l1++ )
        for (int l2 = 0; l2 < num_labels; l2++ )
            smoothTerm[l1+l2*num_labels] = (l1 == l2) ? 0:5;

    try{
        GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width,height,num_labels);
        gc->setDataCost(dataTerm);
        gc->setSmoothCost(smoothTerm);

        printf("\nBefore optimization energy is %lld\n",gc->compute_energy());
        gc->expansion(20);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
        printf("\nAfter optimization energy is %lld\n",gc->compute_energy());

        for(size_t h = 0; h < height; ++h)
            for(size_t w = 0; w < width; ++w)
                lImage.at<uchar>(h + minH, w + minW) = gc->whatLabel(h*width + w);

        delete gc;
    }
    catch (GCException e){
        e.Report();
    }

    delete [] smoothTerm;
    delete [] dataTerm;

    printf("getLabelsFromErrors(): end...\n");
}

#endif //MY3DPHOTO_7LABELSFROMERRORS_H
