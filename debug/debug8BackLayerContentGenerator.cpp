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
#include "8BackLayerContentGenerator.h"

void compute(int argc, char** argv, m3d::Frame &result);
void debug(m3d::Frame &result);
int main(int argc, char** argv)
{
    m3d::Frame result;
    compute(argc, argv, result);
    debug(result);
    return 0;
}

using namespace cv;
using namespace std;

void debug(m3d::Frame &result)
{
    //generate back layers.
    m3d::BackLayerContentGenerator backLayerContentGenerator(result);
    //debug...
    size_t H = m3d::Frame::PANO_H;
    size_t W = m3d::Frame::PANO_W;

    size_t minH = H - 1;
    size_t maxH = 0;
    size_t minW = W - 1;
    size_t maxW = 0;

    for(size_t h = 0; h < H; ++h)
    {
        for(size_t w = 0; w < W; ++w)
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

    cv::Mat &pano_image_f = result.pano_image;
    cv::Mat &pano_depth_f = result.pano_depth;
    cv::Mat &pano_image_b = result.pano_image_b;
    cv::Mat &pano_depth_b = result.pano_depth_b;

    cv::imshow("front image", result.pano_image(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
    cv::imshow("front depth", result.pano_depth(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
    cv::imshow("back image", result.pano_image_b(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
    cv::imshow("back depth", result.pano_depth_b(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
    cv::waitKey();
    //save, or show results.
    //saving to jpg+png rgbd image and rendering from them is faster than rendering from .obj
    //todo
    pano_depth_f.convertTo(pano_depth_f, CV_16UC1);
    pano_depth_b.convertTo(pano_depth_b, CV_16UC1);

    std::string namePrefix("debug8/");
    cv::imwrite(namePrefix + "pano_image_f.jpg", pano_image_f);
    cv::imwrite(namePrefix + "pano_image_b.jpg", pano_image_b);

    cv::imwrite(namePrefix + "pano_depth_f.png", pano_depth_f);
    cv::imwrite(namePrefix + "pano_depth_b.png", pano_depth_b);

    cv::Mat pano_image = cv::Mat(W, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat pano_depth = cv::Mat(W, W, CV_16UC1, cv::Scalar(0));

//    pano_image(cv::Range(0, H), cv::Range::all()) = pano_image_f;
//    pano_image(cv::Range(H, W), cv::Range::all()) = pano_image_b;
    pano_image_f.copyTo(pano_image(cv::Range(0, H), cv::Range::all()));
    pano_image_b.copyTo(pano_image(cv::Range(H, W), cv::Range::all()));

//    pano_depth(cv::Range(0, H), cv::Range::all()) = pano_depth_f;
//    pano_depth(cv::Range(H, W), cv::Range::all()) = pano_depth_b;
    pano_depth_f.copyTo(pano_depth(cv::Range(0, H), cv::Range::all()));
    pano_depth_b.copyTo(pano_depth(cv::Range(H, W), cv::Range::all()));

    cv::imwrite(namePrefix + "pano_image.jpg", pano_image);
    cv::imwrite(namePrefix + "pano_depth.png", pano_depth);
}

void compute(int argc, char** argv, m3d::Frame &result)
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
    m3d::PanoramaStitcher(graph.activatedFrames, frames, result);

}

