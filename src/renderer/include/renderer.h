#ifndef RENDERER_H
#define RENDERER_H

//external
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "spdlog/spdlog.h"
#include <glm/glm.hpp>
//internal
#include "camera.h"

namespace Renderer {
GLFWwindow* CreateGLFWWindow(int width, int height, const char* name);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);
void key_callback(GLFWwindow* window, int key, int scancode, int action,
                  int mods);
}  // namespace Renderer

#endif