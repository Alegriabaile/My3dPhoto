//
// Created by ale on 19-12-7.
//

#include "9Viewer.h"


namespace m3d
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void mouse_callback(GLFWwindow* window, double xpos, double ypos);
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    void processInput(GLFWwindow *window);

    // settings
    const unsigned int SCR_WIDTH = 800;
    const unsigned int SCR_HEIGHT = 600;

    // camera
    CameraForViewer camera;//(glm::vec3(0.0f, 0.0f, 0.0f));
    float lastX = SCR_WIDTH / 2.0f;
    float lastY = SCR_HEIGHT / 2.0f;
    bool firstMouse = true;

    // timing
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    Viewer::Viewer(const std::string &dirName)
    : cameraForViewer(camera)
    {
        GetPanoRgbd(dirName);
        Init();
        draw();
    }

    Viewer::Viewer(const cv::Mat &pano_image_, const cv::Mat &pano_depth_)
    : pano_image(pano_image_.clone()), pano_depth(pano_depth_.clone())
    , cameraForViewer(camera)
    {
        Init();
        draw();
    }

    Viewer::~Viewer() {}

    void Viewer::checkGlError(const std::string step)
    {
        while((err = glGetError()) != GL_NO_ERROR)
        {
            std::cout<<step<<":  err == "<<err<<". \n";
            exit(-1);
        }
    }

    void Viewer::Init()
    {
        InitGlfw();
        InitResources();
    }

    void Viewer::InitGlfw()
    {
        // glfw: initialize and configure
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        // glfw window creation
        window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
        if (window == NULL)
        {
            std::cout << "Viewer::InitGlfw(): Failed to create GLFW window.\n";
            glfwTerminate();
            exit(-1);
        }
        glfwMakeContextCurrent(window);
        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
        glfwSetCursorPosCallback(window, mouse_callback);
        glfwSetScrollCallback(window, scroll_callback);

        // tell GLFW to capture our mouse
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // glad: load all OpenGL function pointers
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Viewer::InitGlfw(): Failed to initialize GLAD.\n";
            exit(-1);
        }
    }

    void Viewer::InitTexture()
    {
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        // load and create a texture
        if(pano_image.empty())
        {
            std::cout<<"Viewer::InitTexture(): texture not load...\n";
            exit(-1);
        }
        cv::flip(pano_image, pano_image, 0);
        //opencv不自动4byte对齐，但是opengl默认传入cpu的数据为4byte对齐
        //        cv::Mat color_tex;
        cv::cvtColor(pano_image, pano_image, CV_BGR2BGRA);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);//Linear for RGB color
        glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                     0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,            // Internal colour format to convert to
                     pano_image.cols,          // Image width  i.e. 640 for Kinect in standard mode
                     pano_image.rows,          // Image height i.e. 480 for Kinect in standard mode
                     0,                 // Border width in pixels (can either be 1 or 0)
                     GL_BGRA, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,  // Image data type
                     pano_image.data);//ptr());        // The actual image data itself
        glGenerateMipmap(GL_TEXTURE_2D);
        //unbind texture
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void Viewer::InitVertices()
    {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    void Viewer::InitShader()
    {
        std::string VertexCode("#version 330 core                                                    \n"
                               "                                                                     \n"
                               "layout (location = 0) in vec3 attributePos;                          \n"
                               "                                                                     \n"
                               "uniform mat4 model;                                                  \n"
                               "uniform mat4 view;                                                   \n"
                               "uniform mat4 projection;                                             \n"
                               "uniform float panoW;                                                 \n"
                               "                                                                     \n"
                               "out vec2 UV;                                                         \n"
                               "                                                                     \n"
                               "void main()                                                          \n"
                               "{                                                                    \n"
                               "    const float PI = 3.1415926535897932384626433832795;              \n"
                               "    float w = attributePos.x;                                        \n"
                               "    float h = attributePos.y;                                        \n"
                               "    float r = attributePos.z;                                        \n"
                               "                                                                     \n"
                               "    float theta = h / panoW * 2.0 * PI;                              \n"
                               "    if(theta > PI)                                                   \n"
                               "        theta = theta - PI;                                          \n"
                               "    float phi = 2.0 * PI - w / panoW * 2.0 * PI;                     \n"
                               "                                                                     \n"
                               "    float x_ = r * sin(theta) * cos(phi);                            \n"
                               "    float y_ = r * sin(theta) * sin(phi);                            \n"
                               "    float z_ = r * cos(theta);                                       \n"
                               "    gl_Position = projection*view*model*vec4(-x_, -z_, -y_, 1.0f);   \n"
                               "                                                                     \n"
                               "    float u = (w + 0.5)/panoW;                                       \n"
                               "    float v = 1.0f - (h + 0.5)/panoW;                                \n"
                               "    UV = vec2(u, v);                                                 \n"
                               "}                                                                    \n");

        std::string FragmentCode("#version 330 core                                                    \n"
                                 "                                                                     \n"
                                 "in vec2 UV;                                                          \n"
                                 "out vec4 FragColor;                                                  \n"
                                 "                                                                     \n"
                                 "uniform sampler2D colorTexture;                                      \n"
                                 "                                                                     \n"
                                 "void main()                                                          \n"
                                 "{                                                                    \n"
                                 "    FragColor = texture(colorTexture, UV.xy);                        \n"
                                 "}                                                                    \n");

        //color
        shaderForViewer.init(VertexCode, FragmentCode);
    }

    void Viewer::InitResources()
    {
        if(pano_image.empty())
        {
            std::cout<<"Viewer::InitResources(): pano_image.empty()!\n";
            exit(-1);
        }
        if(pano_depth.empty())
        {
            std::cout<<"Viewer::InitResources(): pano_depth.empty()!\n";
            exit(-1);
        }
        GetTriangles();
        InitVertices();
        checkGlError("InitVertices()");
        std::cout<<"InitResources(): vertices.size(): "<<vertices.size()<<"\n";

        InitTexture();
        checkGlError("InitTexture()");

        InitShader();
        checkGlError("InitShader()");
    }

    void Viewer::GetPanoRgbd(const std::string &dirName)
    {
        if(dirName.back() == '/')
        {
            pano_image = cv::imread(dirName + "pano_image.jpg");
            pano_depth = cv::imread(dirName + "pano_depth.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        } else
        {
            pano_image = cv::imread(dirName + "/" + "pano_image.jpg");
            pano_depth = cv::imread(dirName + "/" + "pano_depth.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        }

    }

    void Viewer::GetTriangles()
    {
        generateTrianglesFromPanorama(pano_depth, vertices);
    }

    void Viewer::draw()
    {
        //debug...
//        float vecs[9] =



        // configure global opengl state
        glEnable(GL_DEPTH_TEST);
//        glEnable(GL_CULL_FACE);

        shaderForViewer.use();

        // render loop
        while (!glfwWindowShouldClose(window))
        {
            // per-frame time logic
            float currentFrame = glfwGetTime();
            deltaTime = currentFrame - lastFrame;
            lastFrame = currentFrame;
            // input
            processInput(window);

            glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // view/projection transformations
            glm::mat4 projection = glm::perspective(glm::radians(cameraForViewer.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 65500.0f);
            glm::mat4 view = cameraForViewer.GetViewMatrix();
            glm::mat4 model = glm::mat4(1.0f);

            shaderForViewer.setMat4("projection", projection);
            shaderForViewer.setMat4("view", view);
            shaderForViewer.setMat4("model", model);
            shaderForViewer.setFloat("panoW", pano_image.cols);

            //draw................
            // bind textures on corresponding texture units
            glActiveTexture(GL_TEXTURE0);//default actived
            glBindTexture(GL_TEXTURE_2D, texture_id);
            glBindVertexArray(vao);

            glDrawArrays(GL_TRIANGLES, 0, vertices.size()/3);

            glBindVertexArray(0);
            glBindTexture(GL_TEXTURE_2D, 0);
            // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
            // -------------------------------------------------------------------------------
            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        // glfw: terminate, clearing all previously allocated GLFW resources.
        // ------------------------------------------------------------------
        glfwTerminate();
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
    // ---------------------------------------------------------------------------------------------------------
    void processInput(GLFWwindow *window)
    {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(__CameraForViewer::FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(__CameraForViewer::BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(__CameraForViewer::LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(__CameraForViewer::RIGHT, deltaTime);

        if(glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
            camera.ProcessKeyboard(__CameraForViewer::SPEEDUP, deltaTime);
        if(glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
            camera.ProcessKeyboard(__CameraForViewer::SPEEDDOWN, deltaTime);

        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            camera.Reset();

//        printf("processInput\n");
    }

    // glfw: whenever the window size changed (by OS or user resize) this callback function executes
    // ---------------------------------------------------------------------------------------------
    void framebuffer_size_callback(GLFWwindow* window, int width, int height)
    {
        // make sure the viewport matches the new window dimensions; note that width and
        // height will be significantly larger than specified on retina displays.
        glViewport(0, 0, width, height);
    }

    // glfw: whenever the mouse moves, this callback is called
    // -------------------------------------------------------
    void mouse_callback(GLFWwindow* window, double xpos, double ypos)
    {
        if (firstMouse)
        {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

        lastX = xpos;
        lastY = ypos;

        camera.ProcessMouseMovement(xoffset, yoffset);

//        printf("mouse_callback: %lf, %lf \n", xpos, ypos);
    }

    // glfw: whenever the mouse scroll wheel scrolls, this callback is called
    // ----------------------------------------------------------------------
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
    {
        camera.ProcessMouseScroll(yoffset);
    }

    size_t Viewer::generateTrianglesFromPanorama(const cv::Mat &pano_depth_, std::vector<float> &out)
    {
        cv::Mat pano_depth = pano_depth_.clone();
        if(pano_depth.type() != CV_16UC1)
            pano_depth.convertTo(pano_depth, CV_16UC1);

        const float MAX_DIFF = 0.05;//相邻像素深度值可相差的最大比率

        const float H = pano_depth.rows;
        const float W = pano_depth.cols;

        out.clear();
        out.reserve((H - 1) * (W - 1) * 3);//* 6 * 3);

        float x, y, r;//3d coordinates and uv texture coordinates
        for (float h = 0; h < H - 1; ++h) {
            for (float w = 0; w < W - 1; ++w) {
                //参考了 https://github.com/simonfuhrmann/mve/blob/master/libs/mve/depthmap.cc#L211
                //0 1
                //2 3
                float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                d[0] = pano_depth.at<ushort>(h, w);//depth of the current point
                d[1] = pano_depth.at<ushort>((int) h, (int) w + 1);//depth of the right
                d[2] = pano_depth.at<ushort>((int) h + 1, (int) w);//depth of the bottom
                d[3] = pano_depth.at<ushort>((int) h + 1, (int) w + 1);//depth of the right bottom
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

                int tris[4][3] =
                        {
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
                            r = d[tris[i][j]];
                            x = w + tris[i][j] % 2;
                            y = h + tris[i][j] / 2;
//                        x = z * (w + tris[i][j] % 2 - cx) / fx;
//                        y = z * (h + tris[i][j] / 2 - cy) / fy;
//                        u = (w + 0.5 + tris[i][j] % 2) / (float) (W );
//                        v = 1 - (h + 0.5 + tris[i][j] / 2) / (float) (H);
                            out.push_back(x);
                            out.push_back(y);
                            out.push_back(r);
                        }
                    }
                }//end of current(h,w) position's triangle generation

            }//W
        }//H, end of whole loop

        //三角形数目
        return out.size() / (3 * 3);
    }
}