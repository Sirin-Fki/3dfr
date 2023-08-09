#ifndef BFMREADER_H
#define BFMREADER_H

#include <math.h>

#include <glm/glm.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "H5Cpp.h"
#include "data_types.h"
#include "constants.h"

using namespace H5;
using json = nlohmann::json;

class BFM {
   public:
    BFM();
    void readBFMData();

    float* mu_shape;
    float* mu_color;
    float* mu_exp;
    int indices[3 * constants::N_TRIANGLE];
    PCAData shape_pca;
    PCAData color_pca;
    PCAData exp_pca;

    std::vector<Landmark> landmarks;

    int n_landmarks;
    std::vector<float> shape_coeff, color_coeff, exp_coeff;

   private:
    float* readMeanData(std::string model);
    void readIndexData(int cells[]);
    PCAData readPCAData(std::string model);
    void readLandmarks();
    void json2Landmarks(json l);
};

#endif