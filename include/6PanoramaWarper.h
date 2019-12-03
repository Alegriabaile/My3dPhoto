//
// Created by ale on 19-12-2.
//

#ifndef MY3DPHOTO_6PANORAMAWARPER_H
#define MY3DPHOTO_6PANORAMAWARPER_H

#include "0m3d.h"
#include "6OpenglManagerForWarper.h"

namespace m3d
{

    class PanoramaWarper
    {
    private:
        glm::mat4 skyboxModel, skyboxViews[6], skyboxProjection;
        GLuint skyboxVAO, skyboxVBO, skyboxColorTexture, skyboxDepthTexture;


        glm::mat4 panoModels[6], panoView, projection2;
        GLuint panoVAO, panoVBO;


        GLuint inputFramebuffer, inputRenderbuffer;


        double minV, maxV;
        GLenum err;



        OpenglManagerForWarper openglManagerForWarper;

        //从RGBD图片生成致密网格,且相邻像素值在相邻时过大时不生成三角形。未来可将此步骤放入shader中进行优化。
        //vertices: 
        int GenerateTriangles( const cv::Mat &depth,
                               const double * const intrinsics,//intrinsics[4]
                               std::vector<float> &vertices,
                               cv::Mat &radius);
        int GenerateSkybox( const m3d::Frame &frame,
                            const std::vector<float> &vertices);

        int GeneratePoints( std::vector<float> &vPoints);
        int GeneratePanorama( m3d::Frame &frame);

        int InitializeReusable();
        void SKYBOX_GEN_VAO_TEXTURE_MVP(const m3d::Frame &frame, const std::vector<float> &vertices);
        //void FREE_VAO_TEXTURE();

        int WarpToPanorama(m3d::Frame& frame);

    public:

        PanoramaWarper(std::vector<m3d::Frame> & frames);
        ~PanoramaWarper();
    };

}

#endif //MY3DPHOTO_6PANORAMAWARPER_H
