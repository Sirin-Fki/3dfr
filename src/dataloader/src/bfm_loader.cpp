#include "bfm_loader.h"

BFM::BFM() { this->readBFMData(); }

void BFM::readBFMData() {
    this->mu_shape = this->readMeanData("shape");
    this->mu_color = this->readMeanData("color");
    this->mu_exp = this->readMeanData("expression");

    this->readIndexData(this->indices);

    this->shape_pca = this->readPCAData("shape");
    this->color_pca = this->readPCAData("color");
    this->exp_pca = this->readPCAData("expression");

    shape_coeff.resize(constants::N_PCA);
    color_coeff.resize(constants::N_PCA);
    exp_coeff.resize(constants::N_PCA_EXP);

    this->readLandmarks();
}

float* BFM::readMeanData(std::string model) {
    float* mu_raw = new float[constants::N_VERTICE * 3];
    H5File file(constants::bfm_h5_path, H5F_ACC_RDONLY);
    DataSet mu_data = file.openDataSet("/" + model + "/model/mean");
    mu_data.read(mu_raw, PredType::NATIVE_FLOAT);

    mu_data.close();
    file.close();
    return mu_raw;
}

void BFM::readIndexData(int cells[]) {
    int* shape_cells_raw = new int[3 * constants::N_TRIANGLE];

    H5File file(constants::bfm_h5_path, H5F_ACC_RDONLY);
    DataSet shape_cells_data = file.openDataSet("/shape/representer/cells");
    shape_cells_data.read(shape_cells_raw, PredType::NATIVE_INT);
    for (int i = 0; i < 3 * constants::N_TRIANGLE; i++) {
        cells[i] = shape_cells_raw[((i % 3) * constants::N_TRIANGLE) + (i / 3)];
    }
    shape_cells_data.close();
    file.close();
}

PCAData BFM::readPCAData(std::string model) {
    PCAData pca;
    int pca_n = (model == "expression" ? constants::N_PCA_EXP : constants::N_PCA);
    pca.basis.resize(constants::N_VERTICE * 3);
    for (int i = 0; i < constants::N_VERTICE * 3; i++) {
        pca.basis[i].resize(pca_n);
    }

    float* basis = new float[constants::N_VERTICE * 3 * pca_n];
    H5File file(constants::bfm_h5_path, H5F_ACC_RDONLY);
    DataSet basis_data = file.openDataSet("/" + model + "/model/pcaBasis");
    basis_data.read(basis, PredType::NATIVE_FLOAT);

    for (int i = 0; i < constants::N_VERTICE * 3; i++) {
        for (int j = 0; j < pca_n; j++) {
            pca.basis[i][j] = basis[i * pca_n + j];
        }
    }

    float* var = new float[pca_n];
    basis_data.close();
    DataSet var_data = file.openDataSet("/" + model + "/model/pcaVariance");
    var_data.read(var, PredType::NATIVE_FLOAT);
    var_data.close();

    file.close();

    pca.dev.resize(pca_n);
    for (int i = 0; i < pca_n; i++) {
        pca.dev[i] = sqrt(var[i]);
    }
    return pca;
}

bool compareLandmark(Landmark lm1, Landmark lm2) {
    int index1 = std::find(constants::landmarks_ids.begin(),
                           constants::landmarks_ids.end(), lm1.id) -
                 constants::landmarks_ids.begin();
    int index2 = std::find(constants::landmarks_ids.begin(),
                           constants::landmarks_ids.end(), lm2.id) -
                 constants::landmarks_ids.begin();

    return (index1 < index2);
}

void BFM::json2Landmarks(json j) {
    for (int i = 0; i < this->n_landmarks; i++) {
        if (std::find(constants::landmarks_ids.begin(),
                      constants::landmarks_ids.end(),
                      j[i].at("id")) == constants::landmarks_ids.end()) {
            continue;
        }
        Landmark l;
        auto coords = j[i].at("coordinates");
        auto pcvectors = j[i].at("uncertainty").at("pcvectors");
        auto stddevs = j[i].at("uncertainty").at("stddevs");

        l.pos = 0.005f * glm::vec3(coords[0], coords[1], coords[2]);

        l.id = j[i].at("id");
        l.basis.push_back(
            glm::vec3(pcvectors[0][0], pcvectors[0][1], pcvectors[0][2]));
        l.basis.push_back(
            glm::vec3(pcvectors[1][0], pcvectors[1][1], pcvectors[1][2]));
        l.basis.push_back(
            glm::vec3(pcvectors[2][0], pcvectors[2][1], pcvectors[2][2]));

        l.dev = glm::vec3(stddevs[0], stddevs[1], stddevs[2]);

        this->landmarks.push_back(l);
    }
    std::sort(this->landmarks.begin(), this->landmarks.end(), compareLandmark);
}

void BFM::readLandmarks() {
    H5File file(constants::bfm_h5_path, H5F_ACC_RDONLY);
    DataSet landmarks_data = file.openDataSet("/metadata/landmarks/json");
    H5::DataSpace dataspace = landmarks_data.getSpace();
    H5::StrType datatype = landmarks_data.getStrType();
    std::string landmarks_json;
    landmarks_data.read(landmarks_json, datatype, dataspace);
    json j = json::parse(landmarks_json);
    this->n_landmarks = j.size();

    this->json2Landmarks(j);

    landmarks_data.close();
    file.close();
}
