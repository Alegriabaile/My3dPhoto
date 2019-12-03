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
#include "6PanoramaWarper.h"


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
    m3d::PanoramaWarper panoramaWarper(frames);

    //stitch panoramas to result.
    //todo


    //generate back layers.
    //todo


    //save, or show results.
    //saving to jpg+png rgbd image and rendering from them is faster than rendering from .obj
    //todo

}

