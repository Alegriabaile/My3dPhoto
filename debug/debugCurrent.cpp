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
#include "4OverlapEstimator.h"
#include "4GraphInitializer.h"

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

    //debug...
    size_t sz = graph.edges.size();
    cout<<"\t\t\t activated \t inliers \t deltaRotation \t"<<endl;
    for(size_t i=0; i<sz; ++i)
    {
        double gmsInliersNo = graph.edges[i].matches[0];
        double deltaRotation = graph.edges[i].translations[0];//Delta Rotation of angle-axis.
        string act = graph.activatedEdges[i]?"true":"false";
        cout<<i<<" th edge: "<<act<<", "<<gmsInliersNo<<", "<<deltaRotation<<endl;
    }



    //no linear optimization of global pose
    //todo

    //depth deformation
    //todo

    //to be continued
}

