/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_RANKER_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_RANKER_H_

#include <proc_image_processing/algorithm/object_full_data.h>
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

  //==============================================================================
  // I N L I N E   F U N C T I O N S   D E F I N I T I O N S

  //------------------------------------------------------------------------------
  //
  inline bool ObjectRanker::AreaSortFunction(ObjectFullData::Ptr a,
    ObjectFullData::Ptr b) {
    if (a.get() != nullptr && b.get() != nullptr) {
      return a->GetArea() > b->GetArea();
    }
    return false;
  }

  //------------------------------------------------------------------------------
  //
  inline bool ObjectRanker::LengthSortFunction(ObjectFullData::Ptr a,
    ObjectFullData::Ptr b) {
    if (a.get() != nullptr && b.get() != nullptr) {
      return a->GetHeight() > b->GetHeight();
    }
    return false;
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_RANKER_H_
