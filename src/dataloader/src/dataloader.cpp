#include "dataloader.h"

Dataloader::Dataloader(std::string bag_path, int n_frames) {
    this->bfm = new BFM();
    this->rgbd_sequence = importSequence(bag_path, n_frames);
}

BFM* Dataloader::getBFM() { return this->bfm; }
std::vector<Frame*> Dataloader::getRGBD() { return this->rgbd_sequence; }