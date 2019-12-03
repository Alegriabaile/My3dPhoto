//
// Created by ale on 19-12-3.
//
#include <opencv2/opencv.hpp>
#include <iostream>


int main(int argc, char** argv)
{
    using namespace cv;
    using namespace std;

    vector<Mat> rodImages, rodDepths;
    vector<Mat> eulerImages, eulerDepths;

    //read images and depths.
    for(size_t i = 0; i < 14; ++i)
    {
        string rodPrefix("debug6/Rodrigues/"), eulerPrefix("debug6/EulerAngle/");
        string rodImageName(rodPrefix + to_string(i) + "_th_panoImage.jpg");
        string rodDepthName(rodPrefix + to_string(i) + "_th_pano_Depth.png");

        string eulerImageName(eulerPrefix + to_string(i) + "_th_panoImage.jpg");
        string eulerDepthName(eulerPrefix + to_string(i) + "_th_pano_Depth.png");

        Mat tmpImg, tmpDpt;

        tmpImg = imread(rodImageName);
        tmpDpt = imread(rodDepthName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        tmpDpt.convertTo(tmpDpt, CV_32FC1);
        rodImages.push_back(tmpImg);
        rodDepths.push_back(tmpDpt);

        tmpImg = imread(eulerImageName);
        tmpDpt = imread(eulerDepthName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        tmpDpt.convertTo(tmpDpt, CV_32FC1);
        eulerImages.push_back(tmpImg);
        eulerDepths.push_back(tmpDpt);
    }

    //compare the differences.
    for(size_t i = 0; i < 14; ++i)
    {
        size_t width = rodImages[i].cols;
        size_t height = rodImages[i].rows;

        Mat newImage(height, width, CV_8UC3, Scalar(0,0,0));
        Mat newDepth(height, width, CV_32FC1, Scalar(0.0f));
        Mat newLabel(height, width, CV_8UC3, Scalar(0,0,0));

        size_t minH = 10000;
        size_t maxH = 0;
        size_t minW = 10000;
        size_t maxW = 0;
        for(size_t h = 0; h < height; ++h)
        {
            for(size_t w = 0; w < width; ++w)
            {
                float rd = rodDepths[i].at<float>(h,w);
                float ed = eulerDepths[i].at<float>(h,w);
                if( !(rd > 0 ||  ed > 0))
                    continue;

                newImage.at<Vec3b>(h, w)[0] = rodImages[i].at<Vec3b>(h, w)[0];
                newImage.at<Vec3b>(h, w)[2] = eulerImages[i].at<Vec3b>(h, w)[2];

                newDepth.at<float>(h, w) = fabs(rd - ed);

                if(rd > 0)
                    newLabel.at<Vec3b>(h, w)[0] = 255;
                if(ed > 0)
                    newLabel.at<Vec3b>(h, w)[2] = 255;

                if(minH > h)
                    minH = h;
                if(maxH < h)
                    maxH = h;

                if(minW > w)
                    minW = w;
                if(maxW < w)
                    maxW = w;
            }
        }

        Mat image = newImage(Range(minH, maxH), Range(minW, maxW)).clone();
        Mat depth = newDepth(Range(minH, maxH), Range(minW, maxW)).clone();
        Mat label = newLabel(Range(minH, maxH), Range(minW, maxW)).clone();

        imshow("image", image);
        imshow("depth", depth);
        imshow("label", label);
        waitKey();
    }

    return 0;
}