//
// Created by ale on 19-11-8.
//

//
// Created by ale on 19-11-8.
//
#include "0m3d.h"
#include "1FrameReader.h"
#include "2FeatureExtractor.h"
#include "3FeatureMatcher.h"

int main(int argc, char** argv)
{
    using namespace cv;
    using namespace std;
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
    //todo
    m3d::Graph graph;
    m3d::FeatureMatcher::matchFrames(frames, graph);

    //vertice and edge ------> graph ------> initial global pose
    //todo

    //no linear optimization of global pose
    //todo

    //depth deformation
    //todo

    //to be continued



    return 0;
}

