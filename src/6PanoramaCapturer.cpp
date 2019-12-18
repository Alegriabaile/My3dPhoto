//
// Created by ale on 19-12-10.
//

#include "6PanoramaCapturer.h"

namespace m3d
{
    PanoramaCapturer::PanoramaCapturer(const m3d::Graph &graph_, std::vector<m3d::Frame> &frames_)
    : graph(graph_), frames(frames_)
    {
        GeneratePanoramasFromScenes();
    }

    PanoramaCapturer::~PanoramaCapturer() {}

    size_t PanoramaCapturer::GenerateTriangles(const cv::Mat &depth, const double *const intrinsics,
                                             std::vector<float> &vertices, cv::Mat &radius)
    {
        if(radius.empty())
            radius = cv::Mat(depth.size(), CV_32FC1, cv::Scalar(0.0f));
        const float MAX_DIFF = 0.05;//相邻像素深度值可相差的最大比率

        const float rows = depth.rows;
        const float cols = depth.cols;

        const float &fx = intrinsics[0];
        const float &fy = intrinsics[1];
        const float &cx = intrinsics[2];
        const float &cy = intrinsics[3];

        vertices.clear();
        vertices.reserve((rows - 1) * (cols - 1) * 6 * 5);

        float x, y, z, u, v;//3d coordinates and uv texture coordinates
        for (float h = 0; h < rows - 1; ++h)
        {
            for (float w = 0; w < cols - 1; ++w)
            {
                //参考了 https://github.com/simonfuhrmann/mve/blob/master/libs/mve/depthmap.cc#L211
                //0 1
                //2 3
                float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                d[0] = depth.at<float>(h, w);//depth of the current point
                d[1] = depth.at<float>((int) h, (int) w + 1);//depth of the right
                d[2] = depth.at<float>((int) h + 1, (int) w);//depth of the bottom
                d[3] = depth.at<float>((int) h + 1, (int) w + 1);//depth of the right bottom
                //At least three valid depth values are required.
                int positive_d_n = int(d[0] > 0) + int(d[1] > 0) + int(d[2] > 0) + int(d[3] > 0);
                if (positive_d_n < 3) continue;
                //3,1,2;  0,2,3;  3,1,0;  0,2,1
                int tri[4] = {0, 0, 0, 0};
                for (int i = 0; i < 4; ++i)//if positive_d_n==3,then
                {
                    if (!(d[i] > 0)) {
                        tri[i] = 1;
                        break;
                    }
                }
                if (positive_d_n == 4) {
                    if (fabs(d[0] - d[3]) > fabs(d[1] - d[2]))
                        tri[0] = tri[3] = 1;
                    else
                        tri[1] = tri[2] = 1;
                }

                int tris[4][3] = {
                        {3, 1, 2},
                        {0, 2, 3},
                        {3, 1, 0},
                        {0, 2, 1}
                };
                for (int i = 0; i < 4; ++i) {
                    if (tri[i] == 0)
                        continue;

                    int in0 = tris[i][0];
                    int in1 = tris[i][1];
                    int in2 = tris[i][2];
                    if (fabs(d[in0] - d[in1]) < MAX_DIFF * fmax(d[in0], d[in1])
                        && fabs(d[in1] - d[in2]) < MAX_DIFF * fmax(d[in1], d[in2])
                        && fabs(d[in2] - d[in0]) < MAX_DIFF * fmax(d[in2], d[in0])) {
                        for (int j = 0; j < 3; ++j) {
                            z = d[tris[i][j]];
                            x = z * (w + tris[i][j] % 2 - cx) / fx;
                            y = z * (h + tris[i][j] / 2 - cy) / fy;
                            u = (w + 0.5 + tris[i][j] % 2) / (float) (cols);
                            v = 1 - (h + 0.5 + tris[i][j] / 2) / (float) (rows);
                            vertices.push_back(x);
                            vertices.push_back(y);
                            vertices.push_back(z);
                            vertices.push_back(u);
                            vertices.push_back(v);


                            //radius
                        }
                    }
                }//end of current(h,w) position's triangle generation

                z = d[0];
                x = z * (w - cx) / fx;
                y = z * (h - cy) / fy;
                radius.at<float>((size_t)h, (size_t)w) = std::sqrt(x*x + y*y + z*z);
            }
        }//end of whole loop

        //三角形数目
        return vertices.size() / (5 * 3);
    }

    void PanoramaCapturer::transformMatrixFromExtrinsics(const double * const extrinsics, glm::mat4 & transformMat)
    {
        transformMat = glm::mat4(1.0f);
        //relative R, t.
        cv::Mat rVec = (cv::Mat_<double>(3, 1) << extrinsics[0], extrinsics[1], extrinsics[2]);
        cv::Mat R = cv::Mat(3,3,CV_64FC1);
        cv::Rodrigues(rVec, R);

        //glm: matlab type, vertical prior.
        for(size_t h = 0; h < 3; ++h)
            for(size_t w = 0; w < 3; ++w)
                transformMat[w][h] = R.at<double>(h, w);

        for(size_t  i = 0; i < 3; ++i)
            transformMat[3][i] = extrinsics[3+i];

        //wrong
        //从frame.rxryrxtxtytz中恢复此图片的外参，作为model矩阵参数
//        transformMat = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
//
//        transformMat = glm::rotate(transformMat, (float)extrinsics[0], glm::vec3(1.0f, 0.0f, 0.0f));
//        transformMat = glm::rotate(transformMat, (float)extrinsics[1], glm::vec3(0.0f, 1.0f, 0.0f));
//        transformMat = glm::rotate(transformMat, (float)extrinsics[2], glm::vec3(0.0f, 0.0f, 1.0f));
//        transformMat = glm::translate(transformMat, glm::vec3(extrinsics[3], extrinsics[4], extrinsics[5]));//must do translate first!!!
    }

    void PanoramaCapturer::GeneratePanoramasFromScenes()
    {
        m3d::GlfwManagerForCapturer glfwManagerForCapturer;
        m3d::CubemapCapturer cubemapCapturer(m3d::Frame::PANO_H);
        m3d::Cubemap2Sphere cubemap2Sphere(m3d::Frame::PANO_H, m3d::Frame::PANO_W);

        for(size_t i = 0; i < frames.size(); ++i)
            if(graph.activatedFrames[i])
            {
                cv::Mat &color = frames[i].image;
                cv::Mat &depth = frames[i].depth;
                std::vector<float> vertices;
                cv::Mat radius;//use or not to use...
                GenerateTriangles(depth, frames[i].intrinsicD.intrinsic_d(), vertices, radius);

                glm::mat4 modelMat;
                transformMatrixFromExtrinsics(frames[i].extrinsicD.rts, modelMat);
                cubemapCapturer.draw(vertices, color, radius /*or depth?*/, modelMat);

                const GLuint *colorAttaches = cubemapCapturer.colorAttatches();
                const GLuint *depthAttaches = cubemapCapturer.depthAttatches();
                cv::Mat &pano_image = frames[i].pano_image;
                cv::Mat &pano_depth = frames[i].pano_depth;
                cubemap2Sphere.draw(colorAttaches, depthAttaches, pano_image, pano_depth);
            }
    }


}