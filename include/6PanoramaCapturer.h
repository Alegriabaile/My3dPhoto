//
// Created by ale on 19-12-10.
//
#include "0m3d.h"
#include "6GlfwManagerForCapturer.h"
#include "6CubemapCapturer.h"
#include "6Cubemap2Sphere.h"
#ifndef MY3DPHOTO_6PANORAMACAPTURER_H
#define MY3DPHOTO_6PANORAMACAPTURER_H

namespace m3d
{
    class PanoramaCapturer
    {
    private:
        const m3d::Graph &graph;
        std::vector<m3d::Frame> &frames;

        size_t GenerateTriangles( const cv::Mat &depth,
                                  const double * const intrinsics,//intrinsics[4]
                                  std::vector<float> &vertices,
                                  cv::Mat &radius);
        void transformMatrixFromExtrinsics(const double * const extrinsics, glm::mat4 & transformMat);

        void GeneratePanoramasFromScenes();

        void GetPanoBoundary(const cv::Mat &depth, size_t minHwMaxHw[4]);
    public:
        PanoramaCapturer( const m3d::Graph & graph_, std::vector<m3d::Frame> & frames_);

        virtual ~PanoramaCapturer();

    };

}

#endif //MY3DPHOTO_6PANORAMACAPTURER_H
