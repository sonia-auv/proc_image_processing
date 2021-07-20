#include "object_ranker.h"

namespace proc_image_processing {

    void ObjectRanker::rankByArea(ObjectFullData::FullObjectPtrVec objects) {
        std::sort(objects.begin(), objects.end(), ObjectRanker::areaSortFunction);
        auto s = objects.size();
        for (int i = 0; i < s; i++) {
            if (objects[i].get() != nullptr) {
                objects[i]->setAreaRank((float(s - i)) / float(s));
            }
        }
    }

    void ObjectRanker::rankByLength(ObjectFullData::FullObjectPtrVec objects) {
        std::sort(objects.begin(), objects.end(), ObjectRanker::lengthSortFunction);
        auto s = objects.size();
        for (int i = 0; i < s; i++) {
            if (objects[i].get() != nullptr) {
                objects[i]->setLengthRank(float((s - i)) / float(s));
            }
        }
    }

}  // namespace proc_image_processing
