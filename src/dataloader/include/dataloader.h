#include <vector>

#include "bfm_loader.h"
#include "data_types.h"
#include "rgbd.h"

class Dataloader {
   public:
    Dataloader(std::string bag_path, int n_frames);
    BFM* getBFM();
    std::vector<Frame*> getRGBD();

   private:
    BFM* bfm;
    std::vector<Frame*> rgbd_sequence;
};