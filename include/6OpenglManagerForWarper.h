//
// Created by ale on 19-12-2.
//

#ifndef MY3DPHOTO_6OPENGLMANAGERFORWARPER_H
#define MY3DPHOTO_6OPENGLMANAGERFORWARPER_H

#include "6GlfwManagerForWarper.h"
#include "6ShaderForWarper.h"

namespace m3d
{
    class OpenglManagerForWarper
    {
    private:
        GlfwManagerForWarper glfwManagerForWarper;//manage glfw environment.

        GLuint mSkyboxRenderBuffer[6];//, mSkyboxColorRb[6], mSkyboxDepthRb[6];
        GLuint mPanoColorTex2D, mPanoDepthTex2D, mPanoRenderBuffer, mPanoColorRb, mPanoDepthRb;

        int InitializeOpenglResources();
        int GenerateFbos();
        int GenerateShaders();

    public:
        GLuint WIDTH1, HEIGHT1;//
        GLuint WIDTH2, HEIGHT2;//

        GLuint skyboxColorTex2D[6], skyboxDepthTex2D[6];
        GLuint fboSkybox1[6], fboSkybox2[6], fboPano1, fboPano2;
        ShaderForWarper shaderSkybox1, shaderSkybox2, shaderPano1, shaderPano2;

        GLuint attachments[6];

        //
        OpenglManagerForWarper(size_t PANO_H = 2048, size_t PANO_W = 4096);
        ~OpenglManagerForWarper();

    };
}


#endif //MY3DPHOTO_6OPENGLMANAGERFORWARPER_H
