//
// Created by ale on 19-11-22.
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

void DrawMatches(const cv::Mat &src1, const cv::Mat &src2, const std::vector<cv::KeyPoint> &kpt1, const std::vector<cv::KeyPoint> &kpt2, cv::Mat& output, std::size_t type = 1);
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
//        if(!graph.activatedEdges[i])
//            continue;

        const m3d::Edge &edge = graph.edges[i];

        string str1(frames[edge.src].imageFileName), str2(frames[edge.dst].imageFileName);

        cout<<endl;
        cout<<"\t"<<str1.substr(str1.size() - 8)<<"\t"<<str2.substr(str2.size() - 8)<<endl;
        cout<<"\t"<<edge.src<<", "<< edge.dst <<"\tedge.translations[0],[1]: "<<edge.translations[0]<<", "<<edge.translations[1]<<endl;

        cout<<"\t"<<"angle-axis r and translation t: \t";
        for(size_t k = 0; k<6; ++k)
            cout<<edge.rts[k]<<", ";
        cout<<endl;

        if(edge.translations[0] > 0.5 || edge.translations[1] > 1 || edge.translations[0] < 0.01)
            continue;

        double rts[6] = {0};
        PointCloud<PointXYZRGBA>::Ptr cloud2( new PointCloud<PointXYZRGBA>);
        backProject2PclPc(frames[graph.edges[i].dst], rts, cloud2);

        for(size_t k = 0; k<6; ++k)
            rts[k] = graph.edges[i].rts[k];
        PointCloud<PointXYZRGBA>::Ptr cloud1( new PointCloud<PointXYZRGBA>);
        backProject2PclPc(frames[graph.edges[i].src], rts, cloud1);

        double minVal, maxVal;
        cv::minMaxLoc(frames[graph.edges[i].src].depth, &minVal, &maxVal);
        cout<<"\t minVal, maxVal of src: "<<minVal<<", "<<maxVal<<endl;

        *cloud2 += *cloud1;
        viewer2.showCloud( cloud2 );



        const size_t src = graph.edges[i].src;
        const size_t dst = graph.edges[i].dst;

        const std::vector<cv::KeyPoint> &srcPts = graph.edges[i].srcPts;
        const std::vector<cv::KeyPoint> &dstPts = graph.edges[i].dstPts;

        cv::Mat srcImg = frames[src].image.clone();
        cv::Mat dstImg = frames[dst].image.clone();

        cv::Mat output;
        DrawMatches(srcImg, dstImg, srcPts, dstPts, output);

        std::vector<cv::KeyPoint> srcPtsInlier1, dstPtsInlier1;
        for(size_t k = 0; k < graph.edges[i].essentialMask1.rows; ++k)
        {
            if(graph.edges[i].essentialMask1.at<uchar>(k))
            {
                srcPtsInlier1.push_back(srcPts[k]);
                dstPtsInlier1.push_back(dstPts[k]);
            }
        }
        std::vector<cv::KeyPoint> srcPtsInlier2, dstPtsInlier2;
        for(size_t k = 0; k < graph.edges[i].essentialMask2.rows; ++k)
        {
            if(graph.edges[i].essentialMask2.at<uchar>(k))
            {
                srcPtsInlier2.push_back(srcPts[k]);
                dstPtsInlier2.push_back(dstPts[k]);
            }
        }

        cv::Mat output1;
        DrawMatches(srcImg, dstImg, srcPtsInlier1, dstPtsInlier1, output1);
        cv::Mat output2;
        DrawMatches(srcImg, dstImg, srcPtsInlier2, dstPtsInlier2, output2);

        cout<<"\t gms_pts, inlier_pts1, inlier_pts2: "<<srcPts.size()<<", "<<srcPtsInlier1.size()<<", "<<srcPtsInlier2.size()<<std::endl;

        resize(output, output, output.size()/2);
        resize(output1, output1, output1.size()/2);
        resize(output2, output2, output2.size()/2);

        cv::imshow("output", output);
        cv::imshow("output1", output1);
        cv::imshow("output2", output2);

        cv::waitKey();

        cloud1->clear();
        cloud2->clear();
    }


    //no linear optimization of global pose
    //todo

    //depth deformation
    //todo

    //to be continued
}

void DrawMatches(const cv::Mat &src1, const cv::Mat &src2, const std::vector<cv::KeyPoint> &kpt1, const std::vector<cv::KeyPoint> &kpt2, cv::Mat& output, std::size_t type)
{
    const size_t thick_line = 3;
    const size_t radius_circle = 3;
    const size_t thick_circle = 3;

    using namespace cv;
    using namespace std;
    RNG rng;
    if(src1.rows > src1.cols)
    {
        const int height = max(src1.rows, src2.rows);
        const int width = src1.cols + src2.cols;
        output = Mat(height, width, CV_8UC3, Scalar(0, 0, 0));
        src1.copyTo(output(Rect(0, 0, src1.cols, src1.rows)));
        src2.copyTo(output(Rect(src1.cols, 0, src2.cols, src2.rows)));

        for (size_t i = 0; i < kpt1.size(); i++)
        {
            Point2f left = kpt1[i].pt;
            Point2f right = (kpt2[i].pt + Point2f((float)src1.cols, 0.f));

            size_t r = rng(256);
            size_t g = rng(256);
            size_t b = rng(256);
            line(output, left, right, Scalar(b, g, r), thick_line);

            circle(output, left, radius_circle, Scalar(b, g, r), thick_circle);
            circle(output, right, radius_circle, Scalar(b, g, r), thick_circle);
        }
    } else
    {
        const int height = src1.rows + src2.rows;
        const int width = max(src1.cols, src2.cols);
        output = Mat(height, width, CV_8UC3, Scalar(0, 0, 0));
        src1.copyTo(output(Rect(0, 0, src1.cols, src1.rows)));
        src2.copyTo(output(Rect(0, src1.rows, src2.cols, src2.rows)));

        for (size_t i = 0; i < kpt1.size(); i++)
        {
            Point2f left = kpt1[i].pt;
            Point2f right = (kpt2[i].pt + Point2f(0.0f, (float)src1.rows));


            size_t r = rng(256);
            size_t g = rng(256);
            size_t b = rng(256);
            line(output, left, right, Scalar(b, g, r), thick_line);

            circle(output, left, radius_circle, Scalar(b, g, r), thick_circle);
            circle(output, right, radius_circle, Scalar(b, g, r), thick_circle);
        }
    }

}