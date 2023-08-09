#include "constants.h"

std::string lms[19] =
    {/*"jaw.right.down",
     "jaw.left.upper",*/
     "center.nose.tip",           "left.nose.wing.tip",
     "left.nose.hole.center",     "center.nose.attachement_to_philtrum",
     "right.nose.hole.center",    "right.nose.wing.tip",
     "left.eye.corner_outer",     "left.eye.corner_inner",
     "right.eye.corner_inner",    "right.eye.corner_outer",
     "left.lips.philtrum_ridge",  "center.lips.upper.outer",
     "right.lips.philtrum_ridge", "right.lips.corner",
     "center.lips.lower.inner",   "left.lips.corner",
     "center.lips.upper.inner",   "left.eye.pupil.center",
     "right.eye.pupil.center"};

extern std::vector<std::string> constants::landmarks_ids(

    lms, lms + sizeof(lms) / sizeof(lms[0]));

std::string constants::getPCDPath(int i) { return getDataPath(i, true); }

std::string constants::getRGBPath(int i) { return getDataPath(i, false); }

std::string constants::getDataPath(int i, bool pcd) {
    std::stringstream filestream;

#ifdef DATASET
    if (pcd)
        filestream << constants::PCD_DIR << "00" << i <<"_00_cloud.pcd";
    else
        filestream << constants::RGB_DIR << "00" << i << "_00_image.png";
#else
    if (pcd)
        filestream << constants::PCD_DIR << "inputCloud" << i << ".pcd";
    else
        filestream << constants::RGB_DIR << "rgb" << i << ".png";
#endif
    return filestream.str();
}

Eigen::Matrix<float, 3, 4> constants::getProjectionMatrix() {
    Eigen::Matrix<float, 3, 4> P;
#ifdef DATASET
    P << 1052.667867276341f, 0.f, 962.4130834944134f, 0.f, 0.f,
        1052.020917785721f, 536.2206151001486f, 0.f, 0.f, 0.f, 1.f, 0.f;
#else
    //P << 525.f, 0.f, 319.5f, 0.f, 0.f, 525.f, 239.5f, 0.f, 0.f, 0.f, 1.f, 0.f;
    P << 921.404174804688f, 0.f, 637.249816894531, 0.f,
         0.f, 920.25927734375, 355.922973632813f, 0.f,
         0.f, 0.f, 1.f, 0.f;
#endif
    return P;
}
