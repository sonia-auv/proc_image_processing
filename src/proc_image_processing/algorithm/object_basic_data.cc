/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include <proc_image_processing/algorithm/object_basic_data.h>

namespace proc_image_processing {

  ObjectBasicData::ObjectBasicData(const cv::Mat& originalImage,
    const cv::Mat& binaryImage,
    const Contour& contour)
    : area_(0.0f),
    convex_hull_area_(0.0f),
    circumference_(0.0f),
    planes_(NB_OF_PLANE),
    original_image_(originalImage),
    binary_image_(binaryImage),
    vote_count_(0),
    contour_(contour) {
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(AREA, false));
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(CONVEX_HULL, false));
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(CIRCUMFERENCE, false));
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(ROTATED_RECT, false));
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(UP_RIGHT_RECT, false));
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(MOMENTS, false));
    is_calculated_map_.insert(std::pair<OBJECT_DATA, bool>(PLANES, false));

    assert(!originalImage.empty());
    assert(!binaryImage.empty());
  }

  const cv::Mat& ObjectBasicData::GetPlanes(int planesID) {
    if (!is_calculated_map_[PLANES]) {
      cv::Mat gray, hsi;
      planes_.resize(ObjectBasicData::NB_OF_PLANE);
      cv::cvtColor(original_image_, gray, CV_BGR2GRAY);
      cv::cvtColor(original_image_, hsi, CV_BGR2HSV);
      // Set to zeros
      for (int i = 0; i < 7; i++)
        planes_[i] = cv::Mat::zeros(original_image_.rows, original_image_.cols, CV_8UC1);

      cv::split(original_image_, &planes_[0]);
      cv::split(hsi, &planes_[3]);
      gray.copyTo(planes_[6]);
      is_calculated_map_[PLANES] = true;
    }
    // Safety. Should be the constant set in this class, but...
    SetPlaneInRange(planesID);

    return planes_[planesID];
  }

  const cv::Moments& ObjectBasicData::GetMoments(bool useBinary) {
    if (!is_calculated_map_[MOMENTS]) {
      if (useBinary)
        cv_moments_ = cv::moments(binary_image_, useBinary);
      else {
        cv::Mat gray;
        cv::cvtColor(original_image_, gray, CV_BGR2GRAY);
        cv_moments_ = cv::moments(binary_image_, useBinary);
      }
      is_calculated_map_[MOMENTS] = true;
    }
    return cv_moments_;
  }

}  // namespace proc_image_processing
