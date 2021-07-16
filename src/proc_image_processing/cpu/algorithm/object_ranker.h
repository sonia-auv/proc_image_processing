/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_RANKER_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_RANKER_H_

#include "object_full_data.h"
#include <memory>

namespace proc_image_processing {

    // Class that simply rank the object
    // with different value.
    class [[maybe_unused]] ObjectRanker {
    public:
        // sort and set the value in each objects.
        [[maybe_unused]] static void rankByArea(ObjectFullData::FullObjectPtrVec objects);

        [[maybe_unused]] static void rankByLength(ObjectFullData::FullObjectPtrVec objects);

        // Function for std::sort function
        static bool areaSortFunction(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b);

        static bool lengthSortFunction(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b);
    };

    inline bool ObjectRanker::areaSortFunction(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) {
        if (a.get() != nullptr && b.get() != nullptr) {
            return a->getArea() > b->getArea();
        }
        return false;
    }

    inline bool ObjectRanker::lengthSortFunction(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) {
        if (a.get() != nullptr && b.get() != nullptr) {
            return a->getHeight() > b->getHeight();
        }
        return false;
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_RANKER_H_
