/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_

#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/assert.h"
#include "rot_rect.h"
#include "type_and_const.h"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/memory"
#include "contour.h"

namespace proc_image_processing {

class ObjectBasicData {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

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

  //============================================================================
  // P U B L I C   C / D T O R S

  ObjectBasicData(const cv::Mat &originalImage, const cv::Mat &binaryImage,
                  const Contour &contour);

  virtual ~ObjectBasicData() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  void SetPlaneInRange(int &planeID);

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

  const RotRect &GetRotatedRect();

  float GetAngle();

  cv::Point2f &GetCenter();

  const cv::Rect &GetUprightRect();

  const cv::Moments &GetMoments(bool useBinary);

  // Images are already reference in opencv...
  const cv::Mat &GetPlanes(int planesID);

  cv::Mat GetBinaryImageAtUprightRect();

  Contour GetContourCopy();

  cv::Size GetImageSize();

  const cv::Mat &GetBinaryImage();

  const cv::Mat &GetOriginalImage();

 private:
  // P R I V A T E   M E M B E R S

  //============================================================================
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

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void ObjectBasicData::IncrementVote() { vote_count_++; }

//------------------------------------------------------------------------------
//
inline int ObjectBasicData::GetVoteCount() { return vote_count_; }

//------------------------------------------------------------------------------
//
inline void ObjectBasicData::ResetVote() { vote_count_ = 0; }

//------------------------------------------------------------------------------
//
inline float ObjectBasicData::GetArea() {
  if (!is_calculated_map_[AREA]) {
    area_ = cv::contourArea(contour_.GetContour(), false);
    is_calculated_map_[AREA] = true;
  }
  return area_;
}

//------------------------------------------------------------------------------
//
inline float ObjectBasicData::GetHeight() {
  if (!is_calculated_map_[ROTATED_RECT]) {
    rect_ = RotRect(contour_.GetContour());
    is_calculated_map_[ROTATED_RECT] = true;
  }
  return rect_.size.height;
}

//------------------------------------------------------------------------------
//
inline float ObjectBasicData::GetWidth() {
  if (!is_calculated_map_[ROTATED_RECT]) {
    rect_ = RotRect(contour_.GetContour());
    is_calculated_map_[ROTATED_RECT] = true;
  }
  return rect_.size.width;
}

//------------------------------------------------------------------------------
//
inline void ObjectBasicData::SetPlaneInRange(int &planeID) {
  // Clamping the planeID in [0; NB_OF_PLANE - 1]
  planeID =
      planeID < 0 ? 0 : (planeID > NB_OF_PLANE - 1 ? NB_OF_PLANE - 1 : planeID);
}

//------------------------------------------------------------------------------
//
inline float ObjectBasicData::GetConvexHullArea() {
  if (!is_calculated_map_[CONVEX_HULL]) {
    contour_t convexHull;
    cv::convexHull(contour_.GetContour(), convexHull, false, true);
    convex_hull_area_ = cv::contourArea(convexHull, false);
    is_calculated_map_[CONVEX_HULL] = true;
  }
  return convex_hull_area_;
}

//------------------------------------------------------------------------------
//
inline float ObjectBasicData::GetCircumference() {
  if (!is_calculated_map_[CIRCUMFERENCE]) {
    circumference_ = cv::arcLength(contour_.GetContour(), true);
    is_calculated_map_[CIRCUMFERENCE] = true;
  }
  return circumference_;
}

//------------------------------------------------------------------------------
//
inline const RotRect &ObjectBasicData::GetRotatedRect() {
  if (!is_calculated_map_[ROTATED_RECT]) {
    rect_ = RotRect(contour_.GetContour());
    is_calculated_map_[ROTATED_RECT] = true;
  }
  return rect_;
}

//------------------------------------------------------------------------------
//
inline float ObjectBasicData::GetAngle() {
  GetRotatedRect();
  return rect_.angle;
}

//------------------------------------------------------------------------------
//
inline cv::Point2f &ObjectBasicData::GetCenter() {
  GetRotatedRect();
  return rect_.center;
}

//------------------------------------------------------------------------------
//
inline const cv::Rect &ObjectBasicData::GetUprightRect() {
  if (!is_calculated_map_[UP_RIGHT_RECT]) {
    up_right_rect_ = cv::boundingRect(contour_.GetContour());
    is_calculated_map_[UP_RIGHT_RECT] = true;
  }
  return up_right_rect_;
}

//------------------------------------------------------------------------------
//
inline cv::Mat ObjectBasicData::GetBinaryImageAtUprightRect() {
  // Making sure we have calculated the rectangle.
  cv::Rect uprightRect = GetUprightRect();
  // Clone is necessary since the object is created now
  // OpenCV Mat are smart pointer
  return cv::Mat(binary_image_, uprightRect);
}

//------------------------------------------------------------------------------
//
inline Contour ObjectBasicData::GetContourCopy() { return contour_; }

//------------------------------------------------------------------------------
//
inline cv::Size ObjectBasicData::GetImageSize() {
  return original_image_.size();
}

//------------------------------------------------------------------------------
//
inline const cv::Mat &ObjectBasicData::GetBinaryImage() {
  return original_image_;
}

//------------------------------------------------------------------------------
//
inline const cv::Mat &ObjectBasicData::GetOriginalImage() {
  return binary_image_;
}

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_
