//
// Created by ale on 19-11-18.
//

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


void DrawMatches(const Mat &src1, const Mat &src2, const vector<KeyPoint> &kpt1, const vector<KeyPoint> &kpt2, Mat& output, std::size_t type = 1);
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
    m3d::Graph graph;
    m3d::FeatureMatcher::matchFrames(frames, graph);

    //debug...
    const std::size_t sz = graph.edges.size();
    RNG rng(0xFFFFFFFF);
    for(int i=0; i<sz; ++i)
    {
        const size_t src = graph.edges[i].src;
        const size_t dst = graph.edges[i].dst;

        const std::vector<cv::KeyPoint> &srcPts = graph.edges[i].srcPts;
        const std::vector<cv::KeyPoint> &dstPts = graph.edges[i].dstPts;

        cv::Mat srcImg = frames[src].image.clone();
        cv::Mat dstImg = frames[dst].image.clone();

        cv::Mat output;
        DrawMatches(srcImg, dstImg, srcPts, dstPts, output);
        string outputName = "debug3/"+to_string(src)+"_"+to_string(dst)+".jpg";
        //cv::ImwriteFlags ;
        vector<int> paras;
        paras.push_back(IMWRITE_JPEG_QUALITY);
        paras.push_back(100);
        imwrite(outputName, output, paras);
//        resize(output, output, output.size()/2);
//        imshow("output", output);
//        waitKey();
    }

    //vertice and edge ------> graph ------> initial global pose
    //todo

    //no linear optimization of global pose
    //todo

    //depth deformation
    //todo

    //to be continued



    return 0;
}

void DrawMatches(const Mat &src1, const Mat &src2, const vector<KeyPoint> &kpt1, const vector<KeyPoint> &kpt2, Mat& output, std::size_t type)
{
    const size_t thick_line = 3;
    const size_t radius_circle = 3;
    const size_t thick_circle = 3;
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