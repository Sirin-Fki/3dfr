#include "rgbd.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <thread>

#include "stb_image_write.h"

using namespace std::chrono_literals;

std::vector<Frame*> importSequence(std::string bag_path, int n_frames) {
    GLFWwindow* window =
        glfwCreateWindow(1280, 720, "Depthmap alignment", NULL, NULL);
    if (window == NULL) {
        spdlog::error("Failed to create GLFW window");
        glfwTerminate();
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
    }

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device_from_file(bag_path, false);
    auto profile = pipe.start(cfg);
    auto playback = rs2::playback(profile.get_device());
    playback.set_real_time(false);
    Shader shader(constants::RS_FBO_VSHADER, constants::RS_FBO_FSHADER,
                  constants::RS_FBO_GSHADER);
    std::vector<Frame*> sequence;
    for (int i = 0; i < 30; i++) pipe.wait_for_frames();
    for (int i = 0; i < n_frames; i++) {
        auto frame = pipe.wait_for_frames();
        auto depth = frame.get_depth_frame();
        auto color = frame.get_color_frame();
        int width = depth.get_width();
        int height = depth.get_height();

        Frame* f = new Frame(width, height);

        f->depthmap = bilateralFilter(
            alignDepthColor(rsToDepth(depth), shader, width, height), 6.0f,
            0.1f, width, height);

        f->colormap = rsToColor(color);
        computeVertices(f, width, height);
        computeNormals(f, width, height);

        sequence.push_back(f);
        std::cout << "" << std::endl;
    }
    return sequence;
}

// sigmaD distance to the furthest neighbour
float gaussD(float sigma, int x, int y) {
    return exp(-((x * x + y * y) / (2.0f * sigma * sigma)));
}

// sigmaR how big a difference can be between neighbours to affect the weight
float gaussR(float sigma, float dist) {
    return exp(-(dist * dist) / (2.0 * sigma * sigma));
}

std::vector<float> bilateralFilter(std::vector<float> input, float sigmaD,
                                   float sigmaR, int width, int height) {
    std::vector<float> output;
    output.resize(input.size());
    int kernelRadius = (int)ceil(2.0 * sigmaD);
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            float depthCenter = input.at(v * width + u);
            output.at(v * width + u) = 1;
            float sum = 0.0f;
            float sumWeight = 0.0f;
            if (depthCenter != 1) {
                for (int m = u - kernelRadius; m <= u + kernelRadius; m++) {
                    for (int n = v - kernelRadius; n <= v + kernelRadius; n++) {
                        if (m >= 0 && n >= 0 && m < width && n < height) {
                            float currentDepth = input.at(n * width + m);
                            if (currentDepth != 1) {
                                float weight =
                                    gaussD(sigmaD, m - u, n - v) *
                                    gaussR(sigmaR, currentDepth - depthCenter);
                                sumWeight += weight;
                                sum += weight * currentDepth;
                            }
                        }
                    }
                }
                if (sumWeight > 0.0f)
                    output.at(v * width + u) = sum / sumWeight;
            }
        }
    }
    return output;
}

void computeVertices(Frame* f, int width, int height) {
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            Vertex vertex;
            float z = f->depthmap[v * width + u];
            float x = (u - constants::color_intrinsics[0][2]) * z /
                      constants::color_intrinsics[0][0];
            float y = (v - constants::color_intrinsics[1][2]) * z /
                      constants::color_intrinsics[1][1];
            float r = f->colormap[v * 3 * width + u * 3];
            float g = f->colormap[v * 3 * width + u * 3 + 1];
            float b = f->colormap[v * 3 * width + u * 3 + 2];
            vertex.Position = glm::vec3(x, y, z);
            vertex.Color = glm::vec4(r, g, b, 1);
            f->points.push_back(vertex);
        }
    }
}

void computeNormals(Frame* f, int width, int height) {
    glm::vec3 normal = glm::vec3(0, 0, 0);
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (v != 0 && v != height - 1 && u != 0 && u != width - 1) {
                glm::vec3 v1 = f->points[(v - 1) * width + u].Position;
                glm::vec3 v2 = f->points[(v + 1) * width + u].Position;
                glm::vec3 v3 = f->points[v * width + u - 1].Position;
                glm::vec3 v4 = f->points[v * width + u + 1].Position;
                glm::vec3 vertical = v2 - v1;
                glm::vec3 horizontal = v4 - v3;
                Eigen::Vector3d normal =
                    Eigen::Vector3d(vertical.x, vertical.y, vertical.z)
                        .cross(Eigen::Vector3d(horizontal.x, horizontal.y,
                                               horizontal.z));
                normal.normalize();
                f->points[v * width + u].Normal =
                    glm::vec3(normal(0), normal(1), normal(2));
            } else {
                f->points[v * width + u].Normal = glm::vec3(0, 0, 0);
            }
        }
    }
}

float* rsToDepth(rs2::depth_frame depth) {
    int i = 0;
    int height = depth.get_height();
    int width = depth.get_width();

    float* depthmap = new float[height * width];
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            depthmap[i++] = depth.get_distance(u, v);
        }
    }
    return depthmap;
}

std::vector<uint8_t> rsToColor(rs2::video_frame color) {
    std::stringstream color_file;
    color_file << "../../../data/rgb/rgb.png";
    stbi_write_png(color_file.str().c_str(), color.get_width(),
                   color.get_height(), color.get_bytes_per_pixel(),
                   color.get_data(), color.get_stride_in_bytes());
    int i = 0;
    int height = color.get_height();
    int width = color.get_width();

    std::vector<uint8_t> colormap;
    const auto ucolormap = reinterpret_cast<const uint8_t*>(color.get_data());
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            int bytes = u * color.get_bytes_per_pixel();
            int strides = v * color.get_stride_in_bytes();
            int col_index = (bytes + strides);
            colormap.push_back(ucolormap[col_index]);
            colormap.push_back(ucolormap[col_index + 1]);
            colormap.push_back(ucolormap[col_index + 2]);
        }
    }
    return colormap;
}

std::vector<float> alignDepthColor(float* depthmap, Shader shader, int width,
                                   int height) {
    unsigned int VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(depthmap), depthmap, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    unsigned int texture1;
    glGenTextures(1, &texture1);
    glBindTexture(GL_TEXTURE_2D, texture1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT,
                 depthmap);
    glBindTexture(GL_TEXTURE_2D, 0);

    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    unsigned int rendertarget;
    glGenTextures(1, &rendertarget);
    glBindTexture(GL_TEXTURE_2D, rendertarget);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           rendertarget, 0);

    unsigned int renderdepth;
    glGenTextures(1, &renderdepth);
    glBindTexture(GL_TEXTURE_2D, renderdepth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           renderdepth, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        spdlog::error("Framebuffer is not complete!");

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.2f, 0.3f, 0.4f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT || GL_DEPTH_BUFFER_BIT);

    shader.use();

    shader.setMat3("dep_intr", constants::depth_intrinsics);
    shader.setMat3("color_intr", constants::color_intrinsics);
    shader.setMat4("dep_col_extr", constants::depth_color_extrinsics);
    shader.setInt("width", width);
    shader.setInt("height", height);
    shader.setInt("texture1", 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture1);
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, width * height);

    static GLfloat* res = new GLfloat[width * height];
    memset(res, 0, width * height);

    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, res);

    stbi_write_png("../../../data/depth/depth.png", width, height, 4, res,
                   width * 4);
    std::vector<float> res_vec;
    for (int i = 0; i < width * height; i++) {
        res_vec.push_back(res[i]);
    }
    return res_vec;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormals(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_normals_cloud) {
    if (true) {
        pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(
            new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(no_normals_cloud);
        // For every point, use all neighbors in a radius of 3cm.
        normalEstimation.setRadiusSearch(0.03);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(
            new pcl::search::KdTree<pcl::PointXYZRGB>);
        normalEstimation.setSearchMethod(kdtree);

        // Calculate the normals.
        normalEstimation.compute(*normals_ptr);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        pcl::concatenateFields(*no_normals_cloud, *normals_ptr,
                               *cloud_with_normals);
        return cloud_with_normals;
        // pcl::io::savePCDFileASCII((cloud_normals_path).c_str(),
        // *this->cloud);
    } else {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        for (const auto& point : *no_normals_cloud) {
            pcl::PointXYZRGBNormal normal_point;
            normal_point.x = point.x;
            normal_point.y = point.y;
            normal_point.z = point.z;
            normal_point.r = point.r;
            normal_point.g = point.g;
            normal_point.b = point.b;
            normal_point.normal_x = 0;
            normal_point.normal_y = 0;
            normal_point.normal_z = 0;

            cloud_with_normals->push_back(normal_point);
        }
        return cloud_with_normals;
    }
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr visualizeCloud(
    Frame* f, int width, int height, rs2::video_frame color) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(width * height);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(color.get_data());
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            cloud->at(u, v).x = f->points.at(v * width + u).Position.x;
            cloud->at(u, v).y = f->points.at(v * width + u).Position.y;
            cloud->at(u, v).z = f->points.at(v * width + u).Position.z;

            cloud->at(u, v).r = f->points.at(v * width + u).Color.r;
            cloud->at(u, v).g = f->points.at(v * width + u).Color.g;
            cloud->at(u, v).b = f->points.at(v * width + u).Color.b;

            cloud->at(u, v).normal_x = f->points[v * width + u].Normal.x;
            cloud->at(u, v).normal_y = f->points[v * width + u].Normal.y;
            cloud->at(u, v).normal_z = f->points[v * width + u].Normal.z;
        }
    }

    // Visualize cloud and normals
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Normals"));
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, "cloud");
    // Display one normal out of 20, as a line of length 3cm.
    viewer
        ->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(
            cloud, cloud, 20, 0.01, "normals");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return cloud;
}
