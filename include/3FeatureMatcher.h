//
// Created by ale on 19-11-18.
//

#ifndef MY3DPHOTO_3FEATUREMATCHER_H
#define MY3DPHOTO_3FEATUREMATCHER_H

#include "0m3d.h"
#include "gms_matcher_prevent_pcl_ambiguous.h"

namespace m3d
{

    class FeatureMatcher
    {
    private:
        static void matchFeaturePoint(const Frame& frame1, const Frame& frame2, std::size_t src, std::size_t dst, Edge & edge)
        {
            using namespace std;
            using namespace cv;
            const Mat &d1 = frame1.descriptor;
            const Mat &d2 = frame2.descriptor;
            vector<DMatch> matches_all;
            BFMatcher matcher(NORM_HAMMING);

            matcher.match(d1, d2, matches_all);

            // GMS filter
            std::vector<bool> gmsMask;
            const vector<KeyPoint> &kp1 = frame1.keypoints;
            const vector<KeyPoint> &kp2 = frame2.keypoints;
            const cv::Mat &img1 = frame1.image;
            const cv::Mat &img2 = frame2.image;

            gms_matcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);

            int num_inliers = gms.GetInlierMask(gmsMask, false, false);
            cout << "FeatureMatcher::matchFeaturePoint(): "<<endl;
            cout<<"\t\t"<<src<<", "<<dst<<"\t all, inliers:  " <<matches_all.size()<<", "<< num_inliers <<endl;

            edge.matches[0] = num_inliers;
            edge.matches[1] = matches_all.size();

            edge.src = src;
            edge.dst = dst;
            // collect matches
            std::vector<cv::KeyPoint> &srcPts = edge.srcPts;
            std::vector<cv::KeyPoint> &dstPts = edge.dstPts;
            for (size_t i = 0; i < gmsMask.size(); ++i)
            {
                if (gmsMask[i] == true)
                {
                    DMatch dMatch = matches_all[i];
                    srcPts.push_back(kp1[dMatch.queryIdx]);
                    dstPts.push_back(kp2[dMatch.trainIdx]);
                }
            }
        }

    public:
        static void matchFrames(const std::vector<Frame>& frames, Graph& graph)
        {
            std::size_t sz = frames.size();
            graph.resize(sz);

            if(sz>1)
            for(int i=0; i<sz-1; ++i)
                for(int j=i+1; j<sz; ++j)
                {
                    Edge edge;
                    matchFeaturePoint(frames[i], frames[j], i, j, edge);
                    graph.addEdge(edge);
                }

        }
    };


}


#endif //MY3DPHOTO_3FEATUREMATCHER_H
