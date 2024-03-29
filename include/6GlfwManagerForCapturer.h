//
// Created by ale on 19-12-2.
//

#ifndef MY3DPHOTO_6GLFWMANAGERFORCAPTURER_H
#define MY3DPHOTO_6GLFWMANAGERFORCAPTURER_H

#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace m3d
{

    class GlfwManagerForCapturer
    {
    private:


    public:
        GlfwManagerForCapturer()
        {
            glfwInit();
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
            //glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
            // glfw window creation
            GLFWwindow* window = glfwCreateWindow(100, 100, "LearnOpenGL", NULL, NULL);
            if (window == NULL)
            {
                std::cout << "GlfwManagerForCapturer::GlfwManagerForCapturer(): Failed to create GLFW window" << std::endl;

                glfwTerminate();
                exit(-1);
            }
            glfwMakeContextCurrent(window);

            if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            {
                std::cout << "GlfwManagerForCapturer::GlfwManagerForCapturer(): Failed to initialize GLAD" << std::endl;
                exit(-1);
            }

        }

        ~GlfwManagerForCapturer()
        {
            glfwTerminate();
        }

        void checkError()
        {
            GLenum err;
            while((err = glGetError()) != GL_NO_ERROR)
            {
                printf(" GlfwManagerForCapturer::checkError() :    err == %d \n", err);
            }
        }

    };
}


#endif //MY3DPHOTO_6GLFWMANAGERFORCAPTURER_H
