#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <sstream>
#include <string>
#include <vector>

//#define DATASET

namespace constants {
extern std::vector<std::string> landmarks_ids;
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

const float BFM_LMS_RADIUS = 0.01f;
const float PCL_LMS_RADIUS = 0.001f;

const char* const BFM_VSHADER = "../../../src/common/shaders/bfm_shader.vs";
const char* const BFM_FSHADER = "../../../src/common/shaders/bfm_shader.fs";
const char* const BFM_GSHADER = "../../../src/common/shaders/bfm_shader.gs";

const char* const STD_VSHADER = "../../../src/common/shaders/std_shader.vs";
const char* const STD_FSHADER = "../../../src/common/shaders/std_shader.fs";

const char* const PCL_VSHADER = "../../../src/common/shaders/pcl_shader.vs";
const char* const PCL_FSHADER = "../../../src/common/shaders/pcl_shader.fs";

const char* const TEXTURE_VSHADER =
    "../../../src/common/shaders/texture_shader.vs";
const char* const TEXTURE_FSHADER =
    "../../../src/common/shaders/texture_shader.fs";

const char* const BFMTEXTURE_VSHADER = "../../../src/common/shaders/bfm_tex.vs";
const char* const BFMTEXTURE_FSHADER = "../../../src/common/shaders/bfm_tex.fs";
const char* const BFMTEXTURE_GSHADER = "../../../src/common/shaders/bfm_tex.gs";

const char* const RS_FBO_VSHADER = "../../../src/common/shaders/FBO.vs";
const char* const RS_FBO_FSHADER = "../../../src/common/shaders/FBO.fs";
const char* const RS_FBO_GSHADER = "../../../src/common/shaders/FBO.gs";

#ifdef DATASET
const char* const RGB_DIR =
    "/home/sfkiditk/Documents/Studium/Master/4.Semester/IDPs/"
    "3DFaceReconstruction/data/RGBD_Face_dataset_training/"
    "RGBD_Face_dataset_training/";
const char* const PCD_DIR =
    "/home/sfkiditk/Documents/Studium/Master/4.Semester/IDPs/"
    "3DFaceReconstruction/data/RGBD_Face_dataset_training/"
    "RGBD_Face_dataset_training/";
const bool half = true;
#else
const char* const RGB_DIR = "../../../data/rgb/";
const char* const PCD_DIR = "../../../data/depth/";
const bool half = false;
#endif

const char* const BAG_FILE = "../../../data/rgbd_sequence.bag";

const char* const bfm_h5_path =
    "/home/sfkiditk/Documents/Studium/Master/4.Semester/IDPs/"
    "3DFaceReconstruction/data/basel2019/model2019_face12.h5";
const int N_VERTICE = 27657;   // 47439;
const int N_TRIANGLE = 55040;  // 94464;
const int N_PCA = 199;
const int N_PCA_EXP = 100;

const float BFM_SCALE = 0.005f;

const glm::mat3 depth_intrinsics =
    glm::mat3(915.993957519531f, 0.f, 635.271545410156f, 0.f, 915.993957519531f,
              358.46923828125f, 0.f, 0.f, 1.f);
const glm::mat3 color_intrinsics =
    glm::mat3(921.404174804688f, 0.f, 637.249816894531f, 0.f, 920.25927734375f,
              355.922973632813f, 0.f, 0.f, 1.f);
const glm::mat4 depth_color_extrinsics = glm::mat4(
    0.99999f, 0.00140572f, 0.00425911f, 0.0147311044856906f, -0.00140832f,
    0.999999f, 0.000607705f, -0.000159286297275685f, -0.00425825f,
    -0.000613697f, 0.999991f, 0.000156956157297827f, 0.f, 0.f, 0.f, 1.f);

std::string getPCDPath(int i);
std::string getRGBPath(int i);
std::string getDataPath(int i, bool pcd);
Eigen::Matrix<float, 3, 4> getProjectionMatrix();
}  // namespace constants
#endif