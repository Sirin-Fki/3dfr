#include "face_detector.h"

#include "stb_image_write.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY

#include <Eigen/Eigen>

DEFINE_bool(no_display, true, "Enable to disable the visual display.");
void display(
    const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr) {
    try {
        if (datumsPtr != nullptr && !datumsPtr->empty()) {
            const cv::Mat cvMat =
                OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
            if (!cvMat.empty()) {
                cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Face Features",
                           cvMat);
                cv::waitKey(0);
            } else
                op::opLog("Empty cv::Mat as output.", op::Priority::High,
                          __LINE__, __FUNCTION__, __FILE__);
        } else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    } catch (const std::exception& e) {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void configureWrapper(op::Wrapper& opWrapper) {
    try {
        // Configuring OpenPose

        // logging_level
        op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
                      "Wrong logging_level value.", __LINE__, __FUNCTION__,
                      __FILE__);
        op::ConfigureLog::setPriorityThreshold(
            (op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // outputSize
        const auto outputSize =
            op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
        // netInputSize
        const auto netInputSize =
            op::flagsToPoint(op::String("128x128"), "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(
            op::String("256x256"), "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(
            op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(1);
        // poseModel
        const auto poseModel =
            op::flagsToPoseModel(op::String(FLAGS_model_pose));
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::opLog(
                "Flag `write_keypoint` is deprecated and will eventually be "
                "removed. Please, use `write_json`"
                " instead.",
                op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode =
            op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts,
                                                      FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode =
            op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and
        // recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode,
            netInputSize,
            FLAGS_net_resolution_dynamic,
            outputSize,
            keypointScaleMode,
            FLAGS_num_gpu,
            FLAGS_num_gpu_start,
            FLAGS_scale_number,
            (float)FLAGS_scale_gap,
            op::flagsToRenderMode(FLAGS_render_pose, multipleView),
            poseModel,
            !FLAGS_disable_blending,
            (float)FLAGS_alpha_pose,
            (float)FLAGS_alpha_heatmap,
            FLAGS_part_to_show,
            op::String(FLAGS_model_folder),
            heatMapTypes,
            heatMapScaleMode,
            FLAGS_part_candidates,
            (float)FLAGS_render_threshold,
            FLAGS_number_people_max,
            FLAGS_maximize_positives,
            FLAGS_fps_max,
            op::String(FLAGS_prototxt_path),
            op::String(FLAGS_caffemodel_path),
            (float)FLAGS_upsampling_ratio,
            enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            true,
            faceDetector,
            faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView,
                                  FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose,
            (float)FLAGS_face_alpha_heatmap,
            (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand,
            handDetector,
            handNetInputSize,
            FLAGS_hand_scale_number,
            (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView,
                                  FLAGS_render_pose),
            (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap,
            (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to
        // disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking,
            FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose,
            op::String(FLAGS_write_keypoint),
            op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json),
            op::String(FLAGS_write_coco_json),
            FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant,
            op::String(FLAGS_write_images),
            op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video),
            FLAGS_write_video_fps,
            FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps),
            op::String(FLAGS_write_heatmaps_format),
            op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam),
            op::String(FLAGS_write_bvh),
            op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port)};
        opWrapper.configure(wrapperStructOutput);
        // No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
        // Set to single-thread (for sequential processing and/or debugging
        // and/or reducing latency)
        if (FLAGS_disable_multi_thread) opWrapper.disableMultiThreading();
    } catch (const std::exception& e) {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void convertVectorToMat(std::vector<uint8_t> vector, cv::Mat& image, int width,
                        int height) {
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            cv::Vec3b color;
            color[0] = vector.at(v * 3 * width + u * 3 + 2);
            color[1] = vector.at(v * 3 * width + u * 3 + 1);
            color[2] = vector.at(v * 3 * width + u * 3);
            image.at<cv::Vec3b>(v, u) = color;
        }
    }
}

void readLandmarks(
    const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr,
    Frame* f) {
    int lm_n = datumsPtr->at(0)->faceKeypoints.getSize()[1];
    f->landmarks.reserve(lm_n);
    try {
        if (datumsPtr != nullptr && !datumsPtr->empty()) {
            for (int i = 0; i < lm_n * 3; i += 3) {
                float u, v;
                u = datumsPtr->at(0)->faceKeypoints[i];
                v = datumsPtr->at(0)->faceKeypoints[i + 1];

                Landmark l;
                l.index = v * f->width + u;
                l.pos = f->points.at(l.index).Position;

                f->landmarks.push_back(l);
            }
        } else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    } catch (const std::exception& e) {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void DetectFace(Frame* f) {
    cv::Mat mat_image(f->height, f->width, CV_8UC3);
    convertVectorToMat(f->colormap, mat_image, f->width, f->height);
    try {
        op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        configureWrapper(opWrapper);
        opWrapper.start();

        // Process and detect face landmarks from rgb_image
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(mat_image);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
        if (datumProcessed != nullptr) {
            if (!FLAGS_no_display) display(datumProcessed);

        } else
            op::opLog("Image could not be processed.", op::Priority::High);

    } catch (const std::exception&) {
        exit(-1);
    }
}
