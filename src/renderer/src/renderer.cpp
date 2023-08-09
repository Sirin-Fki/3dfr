#include "renderer.h"

using namespace Renderer;

Camera camera(glm::vec3(0.0f, 0.0f, 1.0f));
float lastX = 800 / 2.0f;
float lastY = 600 / 2.0f;
bool firstMouse = true;
bool move_mouse = false;
float deltaTime = 0.0f;
float lastFrame = 0.0f;

GLFWwindow* Renderer::CreateGLFWWindow(int width, int height,
                                       const char* name) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    GLFWwindow* window =
        glfwCreateWindow(mode->width, mode->height, name, NULL, NULL);

    if (window == NULL) {
        spdlog::error("Failed to create GLFW window");
        glfwTerminate();
        exit(-1);
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
        exit(-1);
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    return window;
}

void Renderer::processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

// window size change
void Renderer::framebuffer_size_callback(GLFWwindow* window, int width,
                                         int height) {
    glViewport(0, 0, width, height);
}

// mouse moves
void Renderer::mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (move_mouse) {
        if (firstMouse) {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset =
            lastY - ypos;  // reversed since y-coordinates go from bottom to top

        lastX = xpos;
        lastY = ypos;

        camera.ProcessMouseMovement(xoffset, yoffset);
    }
}

void Renderer::key_callback(GLFWwindow* window, int key, int scancode,
                            int action, int mods) {
    if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        camera.Position = glm::vec3(0.0f, 0.0f, 0.5f);
        camera.WorldUp = glm::vec3(0.0f, 1.0f, 0.0f);
        camera.Yaw = -90.f;
        camera.Pitch = 0.0f;
        camera.updateCameraVectors();
    }
    if (key == GLFW_KEY_M && action == GLFW_PRESS) {
        move_mouse = !move_mouse;
        if (move_mouse)
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        else
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
}

// mouse scroll
void Renderer::scroll_callback(GLFWwindow* window, double xoffset,
                               double yoffset) {
    camera.ProcessMouseScroll(yoffset);
}