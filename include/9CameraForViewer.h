// altered from site of "learnopengl".
//
// Created by ale on 19-8-8.
//

#ifndef NATIVE0701_CAMERA_H
#define NATIVE0701_CAMERA_H


#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>
namespace m3d {
// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
    namespace __CameraForViewer
    {
        enum Camera_Movement {
            FORWARD,
            BACKWARD,
            LEFT,
            RIGHT,
            SPEEDUP,
            SPEEDDOWN
        };

        // Default camera values
        const float YAW = 0.0f;//-90.0f;
        const float PITCH = 0.0f;
        const float SPEED = 100.5f;
        const float SENSITIVITY = 0.01f;
        const float ZOOM = 37.0f;
    }



// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
    class CameraForViewer {
    public:
        // Camera Attributes
        glm::vec3 Position;
        glm::vec3 Front;
        glm::vec3 Up;
        glm::vec3 Right;
        glm::vec3 WorldUp;
        // Euler Angles
        float Yaw;
        float Pitch;
        // Camera options
        float MovementSpeed;
        float MouseSensitivity;
        float Zoom;

        // Constructor with vectors
        CameraForViewer(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
                        glm::vec3 up = glm::vec3(0.0f, -1.0f, 0.0f),
                        glm::vec3 front = glm::vec3(1.0f, 0.0f, 0.0f),
                        float yaw = __CameraForViewer::YAW,
                        float pitch = __CameraForViewer::PITCH)
        {
            Position = position;
            WorldUp = up;
            Front = front;

            Yaw = yaw;
            Pitch = pitch;

            MovementSpeed = __CameraForViewer::SPEED;
            MouseSensitivity = __CameraForViewer::SENSITIVITY;
            Zoom = __CameraForViewer::ZOOM;

            updateCameraVectors();
        }

        // Returns the view matrix calculated using Euler Angles and the LookAt Matrix
        glm::mat4 GetViewMatrix() {
            return glm::lookAt(Position, Position + Front, Up);
            //return glm::lookAt(Position, Front-Position , Up);
        }

        // Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
        void ProcessKeyboard(__CameraForViewer::Camera_Movement direction, float deltaTime) {
//        std::cout<<"deltaTime:"<<deltaTime<<std::endl;
            float velocity = MovementSpeed * deltaTime;
            if (direction == __CameraForViewer::FORWARD)
                Position += Up * velocity;
            //Position += Front * velocity;
            if (direction == __CameraForViewer::BACKWARD)
                Position -= Up * velocity;
            if (direction == __CameraForViewer::LEFT)
                Position -= Right * velocity;
            if (direction == __CameraForViewer::RIGHT)
                Position += Right * velocity;

            const float multi_ = 1.5f;
            if (direction == __CameraForViewer::SPEEDUP)
                MovementSpeed *= multi_;
            if (direction == __CameraForViewer::SPEEDDOWN)
                MovementSpeed = MovementSpeed / multi_;

            if (MovementSpeed < __CameraForViewer::SPEED * (1e-3))
                MovementSpeed = __CameraForViewer::SPEED;
            if (MovementSpeed > __CameraForViewer::SPEED * (1e3))
                MovementSpeed = __CameraForViewer::SPEED;

            updateCameraVectors();
        }

        void ProcessKeyboard(int direction, float deltaTime) {
            float velocity = MovementSpeed * deltaTime;
            if (direction == 0)
                Position += Up * velocity;
            //Position += Front * velocity;
            if (direction == 1)
                Position -= Up * velocity;
            if (direction == 2)
                Position -= Right * velocity;
            if (direction == 3)
                Position += Right * velocity;
        }

        void ResetR() {
            WorldUp = glm::vec3(0.0f, -1.0f, 0.0f);
            Yaw = __CameraForViewer::YAW;
            Pitch = __CameraForViewer::PITCH;
            updateCameraVectors();
        }

        void ResetT() {
            Position = glm::vec3(0.0f, 0.0f, 0.0f);
            updateCameraVectors();
        }

        void Reset() {
            Position = glm::vec3(0.0f, 0.0f, 0.0f);
            WorldUp = glm::vec3(0.0f, -1.0f, 0.0f);
            Yaw = __CameraForViewer::YAW;
            Pitch = __CameraForViewer::PITCH;
            updateCameraVectors();
        }

        // Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
        void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true) {
            xoffset *= MouseSensitivity;
            yoffset *= MouseSensitivity;

            Yaw += xoffset;
            Pitch += yoffset;

            // Make sure that when pitch is out of bounds, screen doesn't get flipped
//        constrainPitch = false;
            if (constrainPitch) {
                //89.0f, -89.0f
                float constranedPitch = 89.0f;
                if (Pitch > constranedPitch)
                    Pitch = constranedPitch;
                if (Pitch < -constranedPitch)
                    Pitch = -constranedPitch;
            }

            // Update Front, Right and Up Vectors using the updated Euler angles
            updateCameraVectors();
        }

        // Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
        void ProcessMouseScroll(float zoffset) {

            if (Zoom >= 1.0f && Zoom <= __CameraForViewer::ZOOM)
                Zoom -= zoffset;
            if (Zoom <= 1.0f)
                Zoom = 1.0f;
            if (Zoom >= __CameraForViewer::ZOOM)
                Zoom = __CameraForViewer::ZOOM;
//        Position += MovementSpeed * Front * zoffset;
        }

    private:
        // Calculates the front vector from the Camera's (updated) Euler Angles
        void updateCameraVectors() {
            // Calculate the new Front vector
            glm::vec3 front;
            front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
            front.y = sin(glm::radians(Pitch));
            front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
            Front = glm::normalize(front);
            // Also re-calculate the Right and Up vector
            Right = glm::normalize(glm::cross(Front,
                                              WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
            Up = glm::normalize(glm::cross(Right, Front));
        }
    };

}
#endif //NATIVE0701_CAMERA_H
