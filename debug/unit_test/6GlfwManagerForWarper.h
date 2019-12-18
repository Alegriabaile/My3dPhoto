//
// Created by ale on 19-12-2.
//

#ifndef MY3DPHOTO_6GLFWMANAGERFORWARPER_H
#define MY3DPHOTO_6GLFWMANAGERFORWARPER_H

#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace m3d
{

    class GlfwManagerForWarper
    {
    private:


    public:
        GlfwManagerForWarper()
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
                std::cout << "GlfwManagerForWarper::GlfwManagerForWarper(): Failed to create GLFW window" << std::endl;

                glfwTerminate();
                exit(-1);
            }
            glfwMakeContextCurrent(window);

            if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            {
                std::cout << "GlfwManagerForWarper::GlfwManagerForWarper(): Failed to initialize GLAD" << std::endl;
                exit(-1);
            }

        }

        ~GlfwManagerForWarper()
        {
            glfwTerminate();
        }

    };
}


#endif //MY3DPHOTO_6GLFWMANAGERFORWARPER_H
