/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_

#include "object_basic_data.h"
#include "object_ranking_data.h"
#include "object_tracking_data.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include "object_feature.h"
#include "general_function.h"

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
    using Ptr = std::shared_ptr<ObjectFullData>;

    typedef std::vector<ObjectFullData::Ptr> FullObjectPtrVec;

    ObjectFullData(const cv::Mat& originalImage, const cv::Mat& binaryImage,
      const Contour& contour);

    virtual ~ObjectFullData() {};
  };

  inline bool AreaSorts(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
    return a->getArea() < b->getArea();
  }

  inline bool RatioSorts(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
    return a->GetRatio() < b->GetRatio();
  }

  cv::Point ObjectVecMedian(ObjectFullData::FullObjectPtrVec);

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_
