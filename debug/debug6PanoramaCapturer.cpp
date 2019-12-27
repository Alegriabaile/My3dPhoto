//
// Created by ale on 19-12-10.
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


void myRun(const std::string appName, const std::string argv1);
void createDir(const std::string &dirName);

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


    //read frame data via the path and state assigned by "argv.txt".
    vector<m3d::Frame> frames;
    m3d::FrameReader(dataDir, state, frames);

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
    //todo
    m3d::PanoramaCapturer PanoramaCapturer(graph, frames);


    //debug
    for(size_t i = 0; i < frames.size(); ++i)
    {
        if(!graph.activatedFrames[i])
            continue;
        std::cout<<i<<" th frame debug...."<<std::endl;

        cv::Mat panoImage = frames[i].pano_image.clone();
        cv::Mat panoDepth = frames[i].pano_depth.clone();

        size_t minH = m3d::Frame::PANO_H - 1;
        size_t maxH = 0;
        size_t minW = m3d::Frame::PANO_W - 1;
        size_t maxW = 0;
        for(size_t h = 0; h < m3d::Frame::PANO_H; ++h)
        {
            for(size_t w = 0; w < m3d::Frame::PANO_W; ++w)
            {
                if(!(panoDepth.at<float>(h,w) > 0))
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
        std::cout<<"minH, maxH, minW, maxW: "<<minH<<", "<<maxH<<", "<<minW<<", "<<maxW<<std::endl;
        printf("frames[%ld].minHwMaxHw: ", i);
        for(size_t k = 0; k < 4; ++k)
            std::cout<<frames[i].minHwMaxHw[k]<<", ";
        std::cout<<std::endl<<std::endl;

        cv::Mat croppedImage = panoImage(cv::Range(minH, maxH), cv::Range(minW, maxW)).clone();

        panoDepth.convertTo(panoDepth, CV_16UC1);

        createDir(dataDir + "/debug6/");
        std::string imgName(dataDir + "/debug6/" + std::to_string(i) + "_th_pano_Image.jpg");
        std::string dptName(dataDir + "/debug6/" + std::to_string(i) + "_th_pano_Depth.png");
        std::string croppedName(dataDir + "/debug6/" + std::to_string(i) + "_th_cropped_Image.jpg");
        cv::imwrite(imgName, panoImage);
        cv::imwrite(dptName, panoDepth);
        cv::imwrite(croppedName, croppedImage);

//        imshow("croppedImage", croppedImage);
//        cv::resize(panoImage, panoImage, cv::Size(1000, 500));
//        cv::resize(panoDepth, panoDepth, cv::Size(1000, 500));
//        imshow("panoImage", panoImage);
//        imshow("panoDepth", panoDepth);
//        waitKey();
    }
}

#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <fstream>


void createDir(const string &dirName)
{
    const __mode_t fullAccess = 7 | 7<<3 | 7<<6;

    if( 0 == access(dirName.c_str(), F_OK))
        cout<<"Path "<<dirName<<" exists"<<endl;
    else
    {
        int state = mkdir(dirName.c_str(), fullAccess);
        if(state == 0)
            cout<<dirName<<" : mkdir succeed!"<<endl;
        else
            cout<<dirName<<" : mkdir failed!"<<endl;
    }
}

