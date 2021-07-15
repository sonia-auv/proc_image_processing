/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_RANKER_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_RANKER_H_

#include "object_full_data.h"
#include <memory>

namespace proc_image_processing {

    // Class that simply rank the object
    // with different value.
    class ObjectRanker {
    public:
        // sort and set the value in each objects.
        static void RankByArea(ObjectFullData::FullObjectPtrVec objects);

        static void RankByLength(ObjectFullData::FullObjectPtrVec objects);

        // Function for std::sort function
        static bool AreaSortFunction(ObjectFullData::Ptr a, ObjectFullData::Ptr b);

        static bool LengthSortFunction(ObjectFullData::Ptr a, ObjectFullData::Ptr b);
    };

    inline bool ObjectRanker::AreaSortFunction(ObjectFullData::Ptr a,
                                               ObjectFullData::Ptr b) {
        if (a.get() != nullptr && b.get() != nullptr) {
            return a->getArea() > b->getArea();
        }
        return false;
    }

    inline bool ObjectRanker::LengthSortFunction(ObjectFullData::Ptr a,
                                                 ObjectFullData::Ptr b) {
        if (a.get() != nullptr && b.get() != nullptr) {
            return a->getHeight() > b->getHeight();
        }
        return false;
    }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_RANKER_H_
