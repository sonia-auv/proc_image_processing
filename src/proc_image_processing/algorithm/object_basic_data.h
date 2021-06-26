/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_

#include <assert.h>
#include <proc_image_processing/algorithm/rot_rect.h>
#include <proc_image_processing/algorithm/type_and_const.h>
#include <memory>
#include "proc_image_processing/algorithm/contour.h"

namespace proc_image_processing {

  class ObjectBasicData {
  public:
    using Ptr = std::shared_ptr<ObjectBasicData>;

    static const int BLUE_PLANE = 0;
    static const int GREEN_PLANE = 1;
    static const int RED_PLANE = 2;
    static const int HUE_PLANE = 3;
    static const int SATURATION_PLANE = 4;
    static const int INTENSITY_PLANE = 5;
    static const int GRAY_PLANE = 6;
    static const int NB_OF_PLANE = 7;

    enum OBJECT_DATA {
      AREA,
      CONVEX_HULL,
      CIRCUMFERENCE,
      ROTATED_RECT,
      UP_RIGHT_RECT,
      MOMENTS,
      PLANES
    };

    ObjectBasicData(const cv::Mat& originalImage, const cv::Mat& binaryImage,
      const Contour& contour);

    virtual ~ObjectBasicData() {}

    void SetPlaneInRange(int& planeID);

    // Voting system
    void IncrementVote();

    int GetVoteCount();

    void ResetVote();

    // All getters calculate their element if they are not already calculated.
    // If they are, simply return them.
    float GetArea();

    float GetHeight();

    float GetWidth();

    float GetConvexHullArea();

    float GetCircumference();

    const RotRect& GetRotatedRect();

    float GetAngle();

    cv::Point2f& GetCenter();

    const cv::Rect& GetUprightRect();

    const cv::Moments& GetMoments(bool useBinary);

    // Images are already reference in opencv...
    const cv::Mat& GetPlanes(int planesID);

    cv::Mat GetBinaryImageAtUprightRect();

    Contour GetContourCopy();

    cv::Size GetImageSize();

    const cv::Mat& GetBinaryImage();

    const cv::Mat& GetOriginalImage();

  private:
    std::map<OBJECT_DATA, bool> is_calculated_map_;

    float area_, convex_hull_area_, circumference_;

    RotRect rect_;
    cv::Rect up_right_rect_;
    cv::Moments cv_moments_;
    std::vector<cv::Mat> planes_;
    cv::Mat original_image_, binary_image_;

    int vote_count_;
    Contour contour_;
  };

  inline void ObjectBasicData::IncrementVote() { vote_count_++; }

  inline int ObjectBasicData::GetVoteCount() { return vote_count_; }

  inline void ObjectBasicData::ResetVote() { vote_count_ = 0; }

  inline float ObjectBasicData::GetArea() {
    if (!is_calculated_map_[AREA]) {
      area_ = cv::contourArea(contour_.GetContour(), false);
      is_calculated_map_[AREA] = true;
    }
    return area_;
  }

  inline float ObjectBasicData::GetHeight() {
    if (!is_calculated_map_[ROTATED_RECT]) {
      rect_ = RotRect(contour_.GetContour());
      is_calculated_map_[ROTATED_RECT] = true;
    }
    return rect_.size.height;
  }

  inline float ObjectBasicData::GetWidth() {
    if (!is_calculated_map_[ROTATED_RECT]) {
      rect_ = RotRect(contour_.GetContour());
      is_calculated_map_[ROTATED_RECT] = true;
    }
    return rect_.size.width;
  }

  inline void ObjectBasicData::SetPlaneInRange(int& planeID) {
    // Clamping the planeID in [0; NB_OF_PLANE - 1]
    planeID =
      planeID < 0 ? 0 : (planeID > NB_OF_PLANE - 1 ? NB_OF_PLANE - 1 : planeID);
  }

  inline float ObjectBasicData::GetConvexHullArea() {
    if (!is_calculated_map_[CONVEX_HULL]) {
      contour_t convexHull;
      cv::convexHull(contour_.GetContour(), convexHull, false, true);
      convex_hull_area_ = cv::contourArea(convexHull, false);
      is_calculated_map_[CONVEX_HULL] = true;
    }
    return convex_hull_area_;
  }

  inline float ObjectBasicData::GetCircumference() {
    if (!is_calculated_map_[CIRCUMFERENCE]) {
      circumference_ = cv::arcLength(contour_.GetContour(), true);
      is_calculated_map_[CIRCUMFERENCE] = true;
    }
    return circumference_;
  }

  inline const RotRect& ObjectBasicData::GetRotatedRect() {
    if (!is_calculated_map_[ROTATED_RECT]) {
      rect_ = RotRect(contour_.GetContour());
      is_calculated_map_[ROTATED_RECT] = true;
    }
    return rect_;
  }

  inline float ObjectBasicData::GetAngle() {
    GetRotatedRect();
    return rect_.angle;
  }

  inline cv::Point2f& ObjectBasicData::GetCenter() {
    GetRotatedRect();
    return rect_.center;
  }

  inline const cv::Rect& ObjectBasicData::GetUprightRect() {
    if (!is_calculated_map_[UP_RIGHT_RECT]) {
      up_right_rect_ = cv::boundingRect(contour_.GetContour());
      is_calculated_map_[UP_RIGHT_RECT] = true;
    }
    return up_right_rect_;
  }

  inline cv::Mat ObjectBasicData::GetBinaryImageAtUprightRect() {
    // Making sure we have calculated the rectangle.
    cv::Rect uprightRect = GetUprightRect();
    // Clone is necessary since the object is created now
    // OpenCV Mat are smart pointer
    return cv::Mat(binary_image_, uprightRect);
  }

  inline Contour ObjectBasicData::GetContourCopy() { return contour_; }

  inline cv::Size ObjectBasicData::GetImageSize() {
    return original_image_.size();
  }

  inline const cv::Mat& ObjectBasicData::GetBinaryImage() {
    return original_image_;
  }

  inline const cv::Mat& ObjectBasicData::GetOriginalImage() {
    return binary_image_;
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_
