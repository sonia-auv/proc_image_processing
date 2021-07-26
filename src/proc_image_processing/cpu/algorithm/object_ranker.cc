/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "object_ranker.h"

namespace proc_image_processing {

  void ObjectRanker::RankByArea(ObjectFullData::FullObjectPtrVec objects) {
    std::sort(objects.begin(), objects.end(), ObjectRanker::AreaSortFunction);
    for (int i = 0, size = objects.size(); i < size; i++) {
      if (objects[i].get() != nullptr) {
        objects[i]->SetAreaRank((float(size - i)) / float(size));
      }
    }
  }

  void ObjectRanker::RankByLength(ObjectFullData::FullObjectPtrVec objects) {
    std::sort(objects.begin(), objects.end(), ObjectRanker::LengthSortFunction);
    for (int i = 0, size = objects.size(); i < size; i++) {
      if (objects[i].get() != nullptr) {
        objects[i]->SetLengthRank(float((size - i)) / float(size));
      }
    }
  }

}  // namespace proc_image_processing
