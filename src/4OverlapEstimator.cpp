//
// Created by ale on 19-11-21.
//
#include "1Logger.h"
#include "4OverlapEstimator.h"

using namespace cv;
using namespace std;

namespace m3d {

    OverlapEstimator::OverlapEstimator(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_)
            : frames(frames_), graph(graph_) {
        CalcInitialRts2d2d();
    }

    OverlapEstimator::~OverlapEstimator() {}

    void OverlapEstimator::CalcInitialRt2d2d(Edge &edge, size_t index) {
        size_t sz = edge.srcPts.size();
        if (sz < 8) {
            LOG("OverlapEstimator::CalcInitialRt2d2d():", to_string(index) + " th matched points.sz < 8");
            return;
        }

        vector<Point2f> pts1, pts2;
        pts1.resize(sz);
        pts2.resize(sz);

        for (size_t i = 0; i < edge.srcPts.size(); ++i) {
            pts1[i] = edge.srcPts[i].pt;
            pts2[i] = edge.dstPts[i].pt;
        }

        Mat mask;
        Mat essential_matrix = findEssentialMat(pts1, pts2, intrinsicK, RANSAC, 0.999, 1.0, mask);
        if (essential_matrix.rows > 3) {
            LOG("OverlapEstimator::CalcInitialRt2d2d():", to_string(index) + " th essential matrix failed...");
            return;
        }

        Mat R, t;
        recoverPose(essential_matrix, pts1, pts2, intrinsicK, R, t, mask);

        Mat rvec = Mat::zeros(3, 1, CV_64FC1);
        Mat tvec = t;///1000.0;//meter.
        Rodrigues(R, rvec);

        for (size_t i = 0; i < 3; ++i) {
            edge.rts[i] = rvec.at<double>(i);
            edge.rts[i + 3] = tvec.at<double>(i);
        }
        edge.translations[0] = min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec));
        edge.translations[1] = cv::norm(tvec);
    }


    void OverlapEstimator::CalcInitialRts2d2d() {
        IntrinsicD &intrinsicD = Frame::intrinsicD;
        intrinsicK = (Mat_<double>(3, 3) <<
                                         intrinsicD.fx_c(), 0, intrinsicD.ppx_c(),
                0, intrinsicD.fy_c(), intrinsicD.ppy_c(),
                0, 0, 1);

        vector<Edge> &edges = graph.edges;
        for (size_t i = 0; i < edges.size(); ++i) {
            CalcInitialRt2d2d(edges[i], i);
        }
    }

}