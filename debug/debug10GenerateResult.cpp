//
// Created by ale on 19-12-16.
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

void myCompute(const std::string appName, const std::string rootDir, const size_t state, m3d::Frame &result);
void mySave(const std::string dataDir, m3d::Frame &result);
void myRun(const std::string appName, const std::string argv1);
int main(int argc, char** argv)
{
    if(argc>2)
        myRun(argv[0], argv[1]);
    else
        myRun(argv[0], "argv.txt");
    return 0;
}


using namespace cv;
using namespace std;

void myRun(const string appName, const string argv1)
{
    std::ifstream inFile(argv1);
    if (!inFile) {
        printf("FrameReader::readArgv:: argv file does not exist.");
        exit(-1);
    }
    string dataDir;
    size_t state;
    inFile >> dataDir >> state;
    inFile.close();

    m3d::Frame result;
    myCompute(appName, dataDir, state, result);
    mySave(dataDir, result);
}
void mySave(const string dataDir, m3d::Frame &result)
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

    cv::Mat pano_image_cropped = result.pano_image(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)).clone();
    cv::Mat pano_depth_cropped = result.pano_depth(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)).clone();

//    cv::imshow("front image", result.pano_image(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
//    cv::imshow("front depth", result.pano_depth(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
//    cv::imshow("back image", result.pano_image_b(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
//    cv::imshow("back depth", result.pano_depth_b(cv::Range(minH, maxH + 1), cv::Range(minW, maxW + 1)));
    cv::imshow("pano_image_cropped", pano_image_cropped);
    cv::imshow("pano_depth_cropped", pano_depth_cropped);
    pano_depth_cropped.convertTo(pano_depth_cropped, CV_16UC1);
    cv::waitKey();
    //save, or show results.
    //saving to jpg+png rgbd image and rendering from them is faster than rendering from .obj
    //todo
    pano_depth_f.convertTo(pano_depth_f, CV_16UC1);
    pano_depth_b.convertTo(pano_depth_b, CV_16UC1);

    std::string namePrefix(dataDir + "/");
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
    cv::imwrite(namePrefix + "pano_image_cropped.jpg", pano_image_cropped);
    cv::imwrite(namePrefix + "pano_depth_cropped.png", pano_depth_cropped);
}

void myCompute(const string appName, const string rootDir, const size_t state, m3d::Frame &result)
{

    //read frame data via the path and state assigned by "argv.txt".
    vector<m3d::Frame> frames;
    m3d::FrameReader(rootDir, state, frames);

    //extract feature points and the responsible descriptors.
    m3d::FeatureExtractor::extractFeatures(frames);

    //robustly match keypoints of image pairs.
    m3d::Graph graph;
    m3d::FeatureMatcher::matchFrames(frames, graph);

    //vertice and edge ------> graph ------> initial global pose
    m3d::OverlapEstimator overlapEstimator(frames, graph);
    m3d::GraphInitializer graphInitializer(frames, graph);

//    m3d::DepthDeformer<2,2> depthDeformer(frames, graph, argv[0]);
    m3d::RigidProblem rigidProblem(frames, graph, appName.c_str());

    //warpToPanorama
    m3d::PanoramaCapturer panoramaCapturer(graph, frames);

    //stitch panoramas to result.
    m3d::PanoramaStitcher(graph.activatedFrames, frames, result);

}


