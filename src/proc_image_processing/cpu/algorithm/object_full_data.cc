/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "object_full_data.h"

namespace proc_image_processing {

  ObjectFullData::ObjectFullData(const cv::Mat& originalImage,
    const cv::Mat& binaryImage,
    const Contour& contour)
    : ObjectBasicData(originalImage, binaryImage, contour) {
  }

  cv::Point ObjectVecMedian(ObjectFullData::FullObjectPtrVec objVec) {
    std::vector<float> xVec;
    std::vector<float> yVec;
    for (auto& elem : objVec) {
      xVec.insert(xVec.begin(), elem->GetCenter().x);
      yVec.insert(yVec.begin(), elem->GetCenter().y);
    }
    return cv::Point((int)(Median(xVec)),
      (int)(Median(yVec)));
  }

}  // namespace proc_image_processing
