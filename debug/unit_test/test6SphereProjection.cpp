//
// Created by ale on 19-12-9.
//

#include "6GlfwManagerForCapturer.h"
#include "6CubemapCapturer.h"
#include "6Cubemap2Sphere.h"

#include "6GlfwManagerForWarper.h"
#include "6OpenglManagerForWarper.h"
#include "6PanoramaWarper.h"
using namespace cv;
using namespace std;


void capture(vector<float> &vertices, Mat& color, Mat& depth);

int main(int argc, char** argv)
{
    std::vector<float> vertices = {
            //front face
            -1, 1, 1, 0.25, 1.0/3.0,
            1, 1, 1, 0.5,  1.0/3.0,
            1, -1, 1, 0.5, 2.0/3.0,
            1, -1, 1, 0.5, 2.0/3.0,
            -1, -1, 1, 0.25, 2.0/3.0,
            -1, 1, 1, 0.25, 1.0/3.0,
            //right face
            1, 1, 1, 0.5, 1.0/3.0,
            1, 1, -1, 0.75, 1.0/3.0,
            1, -1, -1, 0.75, 2.0/3.0,
            1, -1, -1, 0.75, 2.0/3.0,
            1, -1, 1, 0.5, 2.0/3.0,
            1, 1, 1, 0.5, 1.0/3.0,
            //back face
            1, 1, -1, 0.75, 1.0/3.0,
            -1, 1, -1, 1.0, 1.0/3.0,
            -1, -1, -1, 1.0, 2.0/3.0,
            -1, -1, -1, 1.0, 2.0/3.0,
            1, -1, -1, 0.75, 2.0/3.0,
            1, 1, -1, 0.75, 1.0/3.0,
            //left face
            -1, 1, -1, 0.0, 1.0/3.0,
            -1, 1, 1, 0.25, 1.0/3.0,
            -1, -1, 1, 0.25, 2.0/3.0,
            -1, -1, 1, 0.25, 2.0/3.0,
            -1, -1, -1, 0.0, 2.0/3.0,
            -1, 1, -1, 0.0, 1.0/3.0,
            //top face
            -1, -1, 1, 0.25, 2.0/3.0,
            1, -1, 1, 0.5, 2.0/3.0,
            1, -1, -1, 0.5, 1.0,
            1, -1, -1, 0.5, 1.0,
            -1, -1, -1, 0.25, 1.0,
            -1, -1, 1, 0.25, 2.0/3.0,
            //bottom face
            -1, 1, -1, 0.25, 0.0,
            1, 1, -1, 0.5, 0.0,
            1, 1, 1, 0.5, 1.0/3.0,
            1, 1, 1, 0.5, 1.0/3.0,
            -1, 1, 1, 0.25, 1.0/3.0,
            -1, 1, -1, 0.25, 0.0,
    };
    for(size_t i=0; i<vertices.size(); ++i)
    {
        vertices[i] *=1;
    }


    cv::Mat color = cv::imread("test6/cubemap_texture.jpg");//cv::Mat(3, 4, CV_8UC3, cv::Scalar(255, 0, 0));//cv::imread("test6/cubemap.jpg");
    color.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 0, 0);//top
    color.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 255, 0);//left
    color.at<cv::Vec3b>(1, 1) = cv::Vec3b(0, 0, 255);//front
    color.at<cv::Vec3b>(1, 2) = cv::Vec3b(255, 255, 0);//right
    color.at<cv::Vec3b>(1, 3) = cv::Vec3b(0, 255, 255);//back
    color.at<cv::Vec3b>(2, 1) = cv::Vec3b(255, 255, 255);//bottom

//    cv::resize(color, color, cv::Size(400, 300), 0, 0, cv::INTER_NEAREST);
    cv::Mat depth = cv::Mat::eye(10, 10, CV_32FC1);

//    cv::imshow("color", color);
//    cv::waitKey();
    capture(vertices, color, depth);

    return 0;
}

void capture(vector<float> &vertices, Mat& color, Mat& depth)
{
    m3d::GlfwManagerForCapturer glfwManagerForCapturer;
    m3d::CubemapCapturer cubemapCapturer(1024);
    cubemapCapturer.draw(vertices, color, depth);
//    cubemapCapturer.showResults();

    cv::Mat pano_image, pano_depth;
    m3d::Cubemap2Sphere cubemap2Sphere(1024, 2048);
    cubemap2Sphere.draw(cubemapCapturer.colorAttatches(), cubemapCapturer.depthAttatches(), pano_image, pano_depth);
    cubemap2Sphere.showResults();

    cv::imwrite("test6/pano_image.jpg", pano_image);
}