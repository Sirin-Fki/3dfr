
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

#include "data_types.h"
#include "dataloader.h"
#include "face_detector.h"
#include "renderer.h"
#include "spdlog/spdlog.h"

Options setOptions(int argc, char* argv[]) {
    if (argc == 7) {
        return Options(atoi(argv[1]), std::stod(argv[2]), std::stod(argv[3]),
                       atoi(argv[4]), atoi(argv[5]), atoi(argv[6]));
    } else {
        spdlog::warn(
            "No options given - Setting standard options: #Frames = {}, "
            "NN Threshold = {}, NN Angle = {}, ICP Iterations = {}, "
            "Parametric Iterations = {}, Start frame = {}.",
            1, 0.0001, 60, 40, 40, 0);
        return Options(1, 0.0001, 60, 40, 40, 0);
    }
}

int main(int argc, char* argv[]) {
    clock_t start =clock();
    Options opt = setOptions(argc, argv);
    Renderer::CreateGLFWWindow(800, 600, "3D Face Reconstruction");
    Dataloader dataloader(constants::BAG_FILE, 1);
    for (auto f : dataloader.getRGBD()) {
        DetectFace(f);
    }

    clock_t end = clock();
    double elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
    std::cout << elapsed_secs << std::endl;
    std::cout << ""<< std::endl;
    // export_pcl(dataloader.getRGBD()[0], 1280, 720);
    //  Dataloader dataloader();
}