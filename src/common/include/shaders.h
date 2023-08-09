#ifndef SHADERS_H
#define SHADERS_H

#include <glad/glad.h>  // include glad to get all the required OpenGL headers

#include <fstream>
#include <glm/glm.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class Shader {
   public:
    // the program ID
    unsigned int ID;

    // constructor reads and builds the shader
    Shader(const char *vertexPath, const char *fragmentPath,
           const char *geometryPath = nullptr);
    // use/activate the shader
    void use();
    // utility uniform functions
    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec4(const std::string &name, float x, float y, float z, float w) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;
    void setMat3(const std::string &name, const glm::mat3 &mat) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setVec1v(const std::string &name, const std::vector<float> &v) const;
};
#endif