#ifndef DATATYPES_H
#define DATATYPES_H

#include <glm/glm.hpp>
#include <string>
#include <vector>

struct Options {
    int N_FRAMES;
    float nn_threshold;
    float nn_angle;
    int icp_it;
    int param_it;
    int START_FRAME;
    Options(int n, float th, float ang, int icp, int param, int strt)
        : N_FRAMES(n),
          nn_threshold(th),
          nn_angle(ang),
          icp_it(icp),
          param_it(param),
          START_FRAME(strt) {}
};

struct Landmark {
    glm::vec3 pos;
    std::string id;
    int index;
    glm::vec3 dev;
    std::vector<glm::vec3> basis;
};

struct PCAData {
    std::vector<float> dev;
    std::vector<std::vector<float>> basis;
};

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec4 Color;
};

struct Frame {
    std::vector<float> depthmap;
    std::vector<uint8_t> colormap;
    std::vector<Vertex> points;
    std::vector<Landmark> landmarks;
    int width, height;
    Frame(int w, int h) {
        depthmap.reserve(w * h);
        colormap.reserve(w * h * 3);
        points.reserve(w*h);
        landmarks.reserve(0);
        width = w;
        height = h;
    }
};

#endif