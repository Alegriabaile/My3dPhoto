//
// Created by ale on 19-11-21.
//

#ifndef MY3DPHOTO_BACKPROJECT2PCLPC_H
#define MY3DPHOTO_BACKPROJECT2PCLPC_H


#include "0m3d.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv/cxeigen.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>



void backProject2PclPc(m3d::Frame& frame, double *rts, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc)
{
    using namespace cv;
    using namespace pcl;
    //rts[6]: rts[0-2]: r[0-2]; rts[3-5]:  t[3-5]
    //Rotate and Translate matrix.
    cv::Mat rvec = (Mat_<double>(3, 1) << rts[0], rts[1], rts[2]);

    cv::Mat R = Mat(3,3,CV_64FC1);
    cv::Rodrigues(rvec, R);

    int rows = frame.depth.rows;
    int cols = frame.depth.cols;

    double fx, fy, cx, cy;//intrinsics of depth.
    fx = m3d::Frame::intrinsicD.fx_d();
    fy = m3d::Frame::intrinsicD.fy_d();
    cx = m3d::Frame::intrinsicD.ppx_d();
    cy = m3d::Frame::intrinsicD.ppy_d();

    for (int row = 0; row < rows; row = row + 10)
        for (int col = 0; col < cols; col = col + 10)
        {
            double d = frame.depth.ptr<float>(row)[col];
            PointXYZRGBA p;
            double w = d;
            double u = (col - cx) * w / fx;
            double v = (row - cy) * w / fy;

            p.x = R.at<double>(0, 0) * u + R.at<double>(0, 1) * v + R.at<double>(0, 2) * w + rts[3];
            p.y = R.at<double>(1, 0) * u + R.at<double>(1, 1) * v + R.at<double>(1, 2) * w + rts[4];
            p.z = R.at<double>(2, 0) * u + R.at<double>(2, 1) * v + R.at<double>(2, 2) * w + rts[5];

            int hc = round(row*(float)frame.image.rows/(float)rows);
            int wc = round(col*(float)frame.image.cols/(float)cols);
            p.b = frame.image.ptr<uchar>(hc)[wc * 3];
            p.g = frame.image.ptr<uchar>(hc)[wc * 3 + 1];
            p.r = frame.image.ptr<uchar>(hc)[wc * 3 + 2];

            //if(y>-700.0f && y < 1300.0f)
            pc->points.push_back(p);
        }

}


#endif //MY3DPHOTO_BACKPROJECT2PCLPC_H
