//
// Created by ale on 19-12-9.
//

#include "6GlfwManagerForWarper.h"
#include "6CubemapCapturer.h"
#include "6Cubemap2Sphere.h"
//using namespace cv;
//using namespace std;

int main(int argc, char** argv)
{
    std::vector<float> vertices = {
            //front face
            -1, 1, 1, 0.25, 0.33,
            1, 1, 1, 0.5,  0.33,
            1, -1, 1, 0.5, 0.66,
            1, -1, 1, 0.5, 0.66,
            -1, -1, 1, 0.25, 0.66,
            -1, 1, 1, 0.25, 0.33,
            //right face
            1, 1, 1, 0.5, 0.33,
            1, 1, -1, 0.75, 0.33,
            1, -1, -1, 0.75, 0.66,
            1, -1, -1, 0.75, 0.66,
            1, -1, 1, 0.5, 0.66,
            1, 1, 1, 0.5, 0.33,
            //back face
            1, 1, -1, 0.75, 0.33,
            -1, 1, -1, 1.0, 0.33,
            -1, -1, -1, 1.0, 0.66,
            -1, -1, -1, 1.0, 0.66,
            1, -1, -1, 0.75, 0.66,
            1, 1, -1, 0.75, 0.33,
            //left face
            -1, 1, -1, 0.0, 0.33,
            -1, 1, 1, 0.25, 0.33,
            -1, -1, 1, 0.25, 0.66,
            -1, -1, 1, 0.25, 0.66,
            -1, -1, -1, 0.0, 0.66,
            -1, 1, -1, 0.0, 0.33,
            //top face
            -1, -1, 1, 0.25, 0.66,
            1, -1, 1, 0.5, 0.66,
            1, -1, -1, 0.5, 1.0,
            1, -1, -1, 0.5, 1.0,
            -1, -1, -1, 0.25, 1.0,
            -1, -1, 1, 0.25, 0.66,
            //bottom face
            -1, 1, -1, 0.25, 0.0,
            1, 1, -1, 0.5, 0.0,
            1, 1, 1, 0.5, 0.33,
            1, 1, 1, 0.5, 0.33,
            -1, 1, 1, 0.25, 0.33,
            -1, 1, -1, 0.25, 0.0,
    };
    for(size_t i=0; i<vertices.size(); ++i)
    {
        vertices[i] *=1;
    }

    cv::Mat color = cv::imread("test6/cubemap.jpg");
    cv::Mat depth = cv::Mat::ones(512, 1200, CV_32FC1);

    m3d::GlfwManagerForWarper glfwManagerForWarper;
    m3d::CubemapCapturer cubemapCapturer(900);
    cubemapCapturer.draw(vertices, color, depth);
//    cubemapCapturer.showResults();

    cv::Mat pano_image, pano_depth;
    m3d::Cubemap2Sphere cubemap2Sphere(512, 1024);
    cubemap2Sphere.draw(cubemapCapturer.colorAttatches(), cubemapCapturer.depthAttatches(), pano_image, pano_depth);
    cubemap2Sphere.showResults();
    return 0;
}