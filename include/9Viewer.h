//
// Created by ale on 19-12-7.
//

#ifndef MY3DPHOTO_9VIEWER_H
#define MY3DPHOTO_9VIEWER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>
#include "9CameraForViewer.h"
#include "9ShaderForViewer.h"

namespace m3d
{
    class Viewer
    {
    private:
        CameraForViewer &cameraForViewer;
        ShaderForViewer shaderForViewer;
        std::vector<float> vertices;
        cv::Mat pano_image, pano_depth;

        GLFWwindow* window;
        GLenum err;
        GLuint texture_id;
        GLuint vao, vbo;

        void checkGlError(const std::string step);

        void GetPanoRgbd(const std::string &dirName);
        size_t generateTrianglesFromPanorama(const cv::Mat &pano_depth_, std::vector<float> &out);
        void GetTriangles();

        void InitGlfw();
        void InitTexture();
        void InitVertices();
        void InitShader();
        void InitResources();
        void Init();

        void draw();

    public:
        Viewer(const std::string &dirName);
        Viewer(const cv::Mat &pano_image, const cv::Mat &pano_depth);

        virtual ~Viewer();

    };
}

#endif //MY3DPHOTO_9VIEWER_H
