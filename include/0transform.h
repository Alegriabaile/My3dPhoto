//
// Created by ale on 19-11-25.
//

#ifndef MY3DPHOTO_0TRANSFORM_H
#define MY3DPHOTO_0TRANSFORM_H
#include <opencv2/opencv.hpp>

namespace m3d
{
    //globalPose[6], relativePose[6], res[6].

    //assume world pose of src: Rs, ts; world pose of dst: Rd, td.
    //assume relative pose from src to dst: R, t.

    //if globalPose is dst(fSrc2Dst is true).
    //[ R*src + t -> dst.]      [ Rd*dst + td -> world.]        ------->      [ Rd*R*src + Rd*t + td -> world. ]     -------->    R1 = Rd*R, t1 = Rd*t + td.
    //if globalPose is src(fSrc2Dst is false).
    //[ R-1*dst + (-R-1*t) -> src.]    [ Rs*src + ts -> world.]    -------->        [ Rs*R-1*dst + ts - Rs*R-1*t -> world.]        --------> R2 = Rs*R-1, t2 = - Rs*R-1*t + ts.
    void RelativeToGlobal(const double * const globalPose, const double * const relativePose, double res[6], bool fSrc2Dst = true)
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
}

#endif //MY3DPHOTO_0TRANSFORM_H
