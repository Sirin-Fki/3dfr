
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "constants.h"
#include "data_types.h"
#include "shaders.h"
#include "spdlog/spdlog.h"
//#include "stb_image_write.h"

std::vector<uint8_t> rsToColor(rs2::video_frame colormap);
float* rsToDepth(rs2::depth_frame depth);
std::vector<Frame*> importSequence(std::string bag_path, int n_frames);
std::vector<float> alignDepthColor(float* depthmap, Shader shader, int width,
                                     int height);
void computeVertices(Frame* f, int width, int height);
glm::vec3 computeFaceNormal(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4);
void computeNormals(Frame* f, int width, int height);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr visualizeCloud(Frame* f, int width,
                                                  int height,
                                                  rs2::video_frame color);
std::vector<float> bilateralFilter(std::vector<float> input, float sigmaD,
                                   float sigmaR, int width, int height);
float gaussR(float sigma, float dist);
float gaussD(float sigma, int x, int y);
