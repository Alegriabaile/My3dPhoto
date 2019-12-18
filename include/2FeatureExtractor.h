//
// Created by ale on 19-11-8.
//

#ifndef MY3DPHOTO_2FEATUREEXTRACTOR_H
#define MY3DPHOTO_2FEATUREEXTRACTOR_H

#include "0Frame.h"

namespace m3d
{

    //state.id
    //0: default.
    class FeatureExtractor
    {
    private:
        static void extractFeature_(Frame &frame, State state = State())
        {
            std::vector<cv::KeyPoint> &keypoints = frame.keypoints;
            cv::Mat &descriptor = frame.descriptor;
            keypoints.clear();

            cv::Mat &image = frame.image;
            cv::Mat &depth = frame.depth;

            cv::Size size = image.size();
            double scale_min = 0.05;
            //ROI感兴趣区域，即去除一定的边界
            cv::Rect roi(size.width*scale_min, size.height*scale_min, size.width*(1.0-2.0*scale_min), size.height*(1.0-2.0*scale_min));
            cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
            mask(roi).setTo(255);

            cv::Mat depth_resize;
            if(depth.empty())
                depth = cv::Mat(size, CV_32FC1, cv::Scalar(1));
            resize(depth, depth_resize, size);
            mask = (depth_resize > 70.0f) & ( depth_resize < 20000.0f) & mask;

            if(state.id == 0)
            {
                cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
                orb->setFastThreshold(0);
                orb->detectAndCompute(image, mask, keypoints, descriptor);
            }
        }

    public:

        static void extractFeatures(std::vector<Frame> &frames, State state_ = State())
        {
            for(unsigned int i=0; i<frames.size(); ++i)
                extractFeature_(frames[i], state_);
        }
    };
//    void Feature
}


#endif //MY3DPHOTO_2FEATUREEXTRACTOR_H
