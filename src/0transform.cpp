//
// Created by ale on 19-12-2.
//
#include "0transform.h"

namespace m3d
{

    void RelativeToGlobal(const double * const globalPose, const double * const relativePose, double res[6], bool fSrc2Dst)
    {
        //relative R, t.
        cv::Mat rVec = (cv::Mat_<double>(3, 1) << relativePose[0], relativePose[1], relativePose[2]);
        cv::Mat R = cv::Mat(3,3,CV_64FC1);
        cv::Rodrigues(rVec, R);
        cv::Mat t = (cv::Mat_<double>(3, 1) << relativePose[3], relativePose[4], relativePose[5]);

        cv::Mat rVecGlobal = (cv::Mat_<double>(3, 1) << globalPose[0], globalPose[1], globalPose[2]);
        cv::Mat RGlobal = cv::Mat(3,3,CV_64FC1);
        cv::Rodrigues(rVecGlobal, RGlobal);
        cv::Mat tGlobal = (cv::Mat_<double>(3, 1) << globalPose[3], globalPose[4], globalPose[5]);

        cv::Mat RRes = cv::Mat(3,3,CV_64FC1);
        cv::Mat tRes = cv::Mat(3, 1, CV_64FC1);
        if(fSrc2Dst)
        {
            RRes = RGlobal*R;
            tRes = RGlobal*t + tGlobal;
        }else
        {
            cv::Mat R_inv;
            cv::transpose(R, R_inv);
            RRes = RGlobal*R_inv;
            tRes = -RRes*t + tGlobal;
        }

        cv::Mat rVecRes(3, 1, CV_64FC1);
        cv::Rodrigues(RRes, rVecRes);
        for(size_t i=0; i<3; ++i)
        {
            res[i] = rVecRes.at<double>(i);
            res[i+3] = tRes.at<double>(i);
        }
    }

    void transformMatrixFromExtrinsics(const double * const extrinsics, glm::mat4 & transformMat)
    {
        transformMat = glm::mat4(1.0f);
        //relative R, t.
        cv::Mat rVec = (cv::Mat_<double>(3, 1) << extrinsics[0], extrinsics[1], extrinsics[2]);
        cv::Mat R = cv::Mat(3,3,CV_64FC1);
        cv::Rodrigues(rVec, R);

        for(size_t h = 0; h < 3; ++h)
            for(size_t w = 0; w < 3; ++w)
                transformMat[h][w] = R.at<double>(h, w);

        for(size_t  i = 0; i < 3; ++i)
            transformMat[i][3] = extrinsics[3+i];
    }
}