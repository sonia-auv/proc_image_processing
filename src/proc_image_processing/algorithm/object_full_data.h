/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_

#include <proc_image_processing/algorithm/object_basic_data.h>
#include <proc_image_processing/algorithm/object_ranking_data.h>
#include <proc_image_processing/algorithm/object_tracking_data.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include "proc_image_processing/algorithm/object_feature.h"
#include "proc_image_processing/algorithm/general_function.h"

namespace proc_image_processing {

  // Simple container class that is created with the contour.
  // It inherits from the different caracteristic of an object
  // (in the time domain, as a contour and compared to others)
  // It is important to note that it does not calculate all the
  // basic characteristic, it waits until it is ask from the object
  // to calculated. That way, we do not waste calculation time for information
  // we wont use.
  // Also, for tracking and ranking data, it is necessary
  // to fill the object with the help of ObjectRanker and FrameMemory.
  class ObjectFullData : public ObjectTrackingData,
    public ObjectBasicData,
    public ObjectRankingData,
    public ObjectFeatureData {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<ObjectFullData>;

    typedef std::vector<ObjectFullData::Ptr> FullObjectPtrVec;

    //============================================================================
    // P U B L I C   C / D T O R S

    ObjectFullData(const cv::Mat& originalImage, const cv::Mat& binaryImage,
      const Contour& contour);

    virtual ~ObjectFullData() {};
  };

  //==============================================================================
  // I N L I N E   F U N C T I O N S   D E F I N I T I O N S

  //------------------------------------------------------------------------------
  //
  inline bool AreaSorts(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
    return a->GetArea() < b->GetArea();
  }

  //------------------------------------------------------------------------------
  //
  inline bool RatioSorts(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
    return a->GetRatio() < b->GetRatio();
  }

  cv::Point ObjectVecMedian(ObjectFullData::FullObjectPtrVec);

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_
