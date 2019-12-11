//
// Created by ale on 19-12-4.
//

//
// Created by ale on 19-11-25.
//

#include "0m3d.h"
#include "1FrameReader.h"
#include "2FeatureExtractor.h"
#include "3FeatureMatcher.h"
#include "4OverlapEstimator.h"
#include "4GraphInitializer.h"
#include "5DepthDeformer.h"
#include "5PoseNLLSOptimizer.h"
#include "6PanoramaCapturer.h"
#include "7PanoramaStitcher.h"

void run(int argc, char** argv);

int main(int argc, char** argv)
{
    run(argc, argv);
    return 0;
}

#include "backProject2PclPc.h"
using namespace cv;
using namespace std;
using namespace pcl;

void run(int argc, char** argv)
{

//    using namespace m3d;
    string argFileName("argv.txt");
    if(argc>2)
        argFileName.assign(argv[1]);

    //read frame data via the path and state assigned by "argv.txt".
    vector<m3d::Frame> frames;
    m3d::FrameReader(argFileName, frames);

    //extract feature points and the responsible descriptors.
    m3d::FeatureExtractor::extractFeatures(frames);

    //robustly match keypoints of image pairs.
    m3d::Graph graph;
    m3d::FeatureMatcher::matchFrames(frames, graph);

    //vertice and edge ------> graph ------> initial global pose
    m3d::OverlapEstimator overlapEstimator(frames, graph);
    m3d::GraphInitializer graphInitializer(frames, graph);

//    m3d::DepthDeformer<2,2> depthDeformer(frames, graph, argv[0]);
    m3d::RigidProblem rigidProblem(frames, graph, argv[0]);

    //warpToPanorama
    m3d::PanoramaCapturer panoramaCapturer(graph, frames);

    //stitch panoramas to result.
    m3d::Frame result;
    m3d::PanoramaStitcher(graph.activatedFrames, frames, result);

    //debug...

//    for(size_t i = 0; i < frames.size(); ++i)
//    {
//        const cv::Mat &pano_image = frames[i].pano_image;
//        const cv::Mat &pano_depth = frames[i].pano_depth;
//        const cv::Mat &pano_error = frames[i].pano_error;
//
//        size_t minH = m3d::Frame::PANO_H - 1;
//        size_t maxH = 0;
//        size_t minW = m3d::Frame::PANO_W - 1;
//        size_t maxW = 0;
//
//        for(size_t h = 0; h < m3d::Frame::PANO_H; ++h)
//        {
//            for(size_t w = 0; w < m3d::Frame::PANO_W; ++w)
//            {
//                if(!(pano_depth.at<float>(h,w) > 0))
//                    continue;
//
//                if(minH > h)
//                    minH = h;
//                if(maxH < h)
//                    maxH = h;
//
//                if(minW > w)
//                    minW = w;
//                if(maxW < w)
//                    maxW = w;
//            }
//        }
//
//        cv::Mat image = pano_image(Range(minH, maxH), Range(minW, maxW)).clone();
//        cv::Mat error = pano_error(Range(minH, maxH), Range(minW, maxW)).clone();
//
//        cv::imshow("image", image);
//        cv::imshow("error", error/5.0f);
//        cv::waitKey();
//    }

    std::cout<<"start debug..."<<std::endl;

    size_t minH = m3d::Frame::PANO_H - 1;
    size_t maxH = 0;
    size_t minW = m3d::Frame::PANO_W - 1;
    size_t maxW = 0;

    for(size_t h = 0; h < m3d::Frame::PANO_H; ++h)
    {
        for(size_t w = 0; w < m3d::Frame::PANO_W; ++w)
        {
            if(!(result.pano_depth.at<float>(h,w) > 0))
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

    cv::imshow("result: image", result.pano_image(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
//    cv::imshow("result: depth", result.pano_depth(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
//    cv::imshow("result: error", result.pano_error(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
//    cv::imshow("result: label", result.pano_label(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1))*3000);

    cv::imwrite("debug7/result.pano_image.jpg", result.pano_image);
    cv::waitKey();


    //generate back layers.
    //todo


    //save, or show results.
    //saving to jpg+png rgbd image and rendering from them is faster than rendering from .obj
    //todo

}

