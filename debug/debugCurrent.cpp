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
    //todo
    m3d::Graph graph;
    m3d::FeatureMatcher::matchFrames(frames, graph);

    //vertice and edge ------> graph ------> initial global pose
    //todo
    m3d::OverlapEstimator overlapEstimator(frames, graph);


    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////debug///////////////////////////////
    //////////////////////////////////////////////////////////////////////////
//    pcl::visualization::CloudViewer viewer1("pcl viewer1");
    pcl::visualization::CloudViewer viewer2("pcl viewer2");

    size_t sz = graph.edges.size();
    bool stopFlag = false;
    for(size_t i = 0; i<sz; ++i)
    {
        const m3d::Edge &edge = graph.edges[i];

        cout<<"\t"<<frames[edge.src].imageFileName<<"\t"<<frames[edge.dst].imageFileName<<endl;
        cout<<"\t"<<edge.src<<", "<< edge.dst <<"\tedge.translations[0],[1]: "<<edge.translations[0]<<", "<<edge.translations[1]<<endl;
        for(size_t k = 0; k<6; ++k)
            cout<<edge.rts[k]<<", ";
        cout<<endl;

        if(edge.translations[0] > 0.5 || edge.translations[1] > 1 || edge.translations[0] < 0.01)
            continue;

        double rts[6] = {0};

        PointCloud<PointXYZRGBA>::Ptr cloud1( new PointCloud<PointXYZRGBA>);
        backProject2PclPc(frames[graph.edges[i].src], rts, cloud1);
        double minVal, maxVal;
        cv::minMaxLoc(frames[graph.edges[i].src].depth, &minVal, &maxVal);


        for(size_t k = 0; k<6; ++k)
            rts[k] = -graph.edges[i].rts[k];

        PointCloud<PointXYZRGBA>::Ptr cloud2( new PointCloud<PointXYZRGBA>);
        backProject2PclPc(frames[graph.edges[i].dst], rts, cloud2);

        *cloud2 += *cloud1;

//        viewer1.showCloud( cloud1 );
        cv::imshow("img1", frames[graph.edges[i].src].image);
        cv::imshow("img2", frames[graph.edges[i].dst].image);
        viewer2.showCloud( cloud2 );
        while( !stopFlag )
        {
            if(cv::waitKey(10) == ' ')
                stopFlag = true;
        }
        stopFlag = false;
        cloud1->clear();
        cloud2->clear();
    }


    //no linear optimization of global pose
    //todo

    //depth deformation
    //todo

    //to be continued
}

