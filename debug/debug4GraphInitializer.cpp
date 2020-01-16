//
// Created by ale on 19-11-25.
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



    std::cout<<"start GraphInitializer..."<<std::endl;
    m3d::GraphInitializer graphInitializer(frames, graph);


    std::cout<<"start debug..."<<std::endl;
    //debug...

    //debug...
    for(size_t i = 0; i < graph.edges.size(); ++i)
    {
        if(!graph.activatedEdgesMst[i])
            continue;

        std::cout<<"\t"<<i<<" th activated from mst---- src, dst, rts[6]:  "<<std::endl<<"\t";

        size_t src = graph.edges[i].src;
        size_t dst = graph.edges[i].dst;
        double *rts = graph.edges[i].rts;
        std::cout<<src<<", "<<dst<<" : ";
        for(size_t j = 0; j < 6; ++j)
            std::cout<<rts[j]<<" ";
        std::cout<<std::endl;
    }

    pcl::visualization::CloudViewer viewer("pcl viewer");
    const std::vector<bool> &activatedFrames = graph.activatedFrames;
    const size_t sz = frames.size();
    PointCloud<PointXYZRGBA>::Ptr cloud1( new PointCloud<PointXYZRGBA>);
    cv::imshow("control", cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255)));
    for(size_t i = 0; i<sz; ++i)
    {
        if(!activatedFrames[i])
            continue;

        string str1(frames[i].imageFileName);

        cout<<endl;
        cout<<"\t"<<i<<" th frame:\t"<<str1.substr(str1.size() - 8)<<endl;

        double *rts = frames[i].extrinsicD.rts;
        for(size_t k = 0; k < 6; ++k)
            cout<<"\t"<<rts[k]<<", ";
        cout<<endl;

        PointCloud<PointXYZRGBA>::Ptr cloud2( new PointCloud<PointXYZRGBA>);
        backProject2PclPc(frames[i], rts, cloud2);

        double minVal, maxVal;
        cv::minMaxLoc(frames[i].depth, &minVal, &maxVal);
        cout<<"\t minVal, maxVal of src: "<<minVal<<", "<<maxVal<<endl;

        *cloud1 += *cloud2;
        cloud2->clear();

        cv::waitKey(3000);
        viewer.showCloud(cloud1);
    }
    
    while(!viewer.wasStopped())
        ;
    cloud1->clear();

    printf("absolute rts:\n");
    for(size_t k = 0; k < frames.size(); ++k)
    {
        printf("%ld ", k);
        for(size_t i = 0; i < 6; ++i)
            printf("%lf, ", frames[k].extrinsicD.rts[i]);
        printf("\n");
    }
    printf("relative rts:\n");
    for(size_t k = 0; k < graph.edges.size(); ++k)
    {
        if(!graph.activatedEdgesMst[k])
            continue;
        int src = graph.edges[k].src;
        int dst = graph.edges[k].dst;
        printf("%d %d ", src, dst);
        for(size_t i = 0; i < 6; ++i)
            printf("%lf, ", graph.edges[k].rts[i]);
        printf("\n");
    }
    //no linear optimization of global pose
    //todo

    //depth deformation
    //todo

    //to be continued
}

