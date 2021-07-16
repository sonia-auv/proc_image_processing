/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "object_ranker.h"

namespace proc_image_processing {

    [[maybe_unused]] void ObjectRanker::rankByArea(ObjectFullData::FullObjectPtrVec objects) {
        std::sort(objects.begin(), objects.end(), ObjectRanker::areaSortFunction);
        for (int i = 0, size = objects.size(); i < size; i++) {
            if (objects[i].get() != nullptr) {
                objects[i]->setAreaRank((float(size - i)) / float(size));
            }
        }
    }

    [[maybe_unused]] void ObjectRanker::rankByLength(ObjectFullData::FullObjectPtrVec objects) {
        std::sort(objects.begin(), objects.end(), ObjectRanker::lengthSortFunction);
        for (int i = 0, size = objects.size(); i < size; i++) {
            if (objects[i].get() != nullptr) {
                objects[i]->setLengthRank(float((size - i)) / float(size));
            }
        }
    }

}  // namespace proc_image_processing
