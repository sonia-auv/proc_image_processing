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

#ifndef PROVIDER_VISION_FILTERS_BUOY_SINGLE_H_
#define PROVIDER_VISION_FILTERS_BUOY_SINGLE_H_

#include <math.h>
#include <proc_image_processing/cpu/algorithm/contour_list.h>
#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/object_feature_factory.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include "filter.h"
#include <proc_image_processing/cpu/server/target.h>
#include <memory>

namespace proc_image_processing {

class BuoySingle : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BuoySingle>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit BuoySingle(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        feat_factory_(1),
        enable_("Enable", false, &parameters_),
        debug_good_contour_("Debug_contour", false, &parameters_),
        eliminate_same_x_targets_("Eliminate_same_x", false, &parameters_),
        detect_red_("Detect_red", false, &parameters_),
        color_("Buoy", "red", &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_),
        max_ratio_("Max_ratio", 50, 0, 100, &parameters_),
        min_filled_percent_("Min_percent", 50, 0, 100, &parameters_),
        max_x_difference_for_elimination_("Min_x_difference", 50.0f, 0.0f,
                                          1000.0f, &parameters_),
        ratio_for_angle_check_(
            "Ratio_for_angle_check", 50, 0, 100, &parameters_,
            "When ratio smaller, will discard if vertical contour"),
        admissible_horizontal_angle_(
            "horizontal_max_angle", 50, 0, 90, &parameters_,
            "When angle smaller, is consider vertical") {
    SetName("BuoySingle");
  }

  virtual ~BuoySingle() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      cv::Mat in;
      // -----------------   Setup the debug image
      // Copy the received image so we can see the objects.
      if (debug_good_contour_()) {
        // Case we receive a color or gray scale image.
        if (image.channels() == 1)
          cv::cvtColor(image, output_image_, CV_GRAY2BGR);
        else
          image.copyTo(output_image_);
      }

      if (image.channels() > 1) {
        cv::cvtColor(image, in, CV_BGR2GRAY);
      } else {
        image.copyTo(in);
      }

      //-----------------------
      // find contours in the image
      ContourList contours(image, ContourList::OUTER);

      // Filter contours
      ObjectFullData::FullObjectPtrVec objectVec;

      for (size_t j = 0; j < contours.GetSize(); j++) {
        if (contours[j].GetSize() <= 2) {
          continue;
        }

        ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(
            global_params_.getOriginalImage(), image, contours[j]);

        // AREA
        if (min_area_() > object->GetArea()) {
          continue;
        }

        if (debug_good_contour_()) {
          contours[j].DrawContours(output_image_, CV_RGB(255, 0, 0), 3);
        }
        feat_factory_.RatioFeature(object);
        // RATIO
        if (max_ratio_() > object->GetRatio()) {
          continue;
        }

        if (debug_good_contour_()) {
          contours[j].DrawContours(output_image_, CV_RGB(255, 255, 0), 3);
        }

        // Angle of contour. If more rectangular, check to make sure it is
        // horizontal
        // not vertical, horizontal is caused by washed out buoy in the sun,
        // while
        // vertical is probably a pipe
        if (object->GetAngle() < ratio_for_angle_check_() &&
            fabs(object->GetAngle()) < admissible_horizontal_angle_()) {
          continue;
        }

        if (debug_good_contour_()) {
          contours[j].DrawContours(output_image_, CV_RGB(255, 0, 255), 3);
        }

        feat_factory_.PercentFilledFeature(object);
        if (min_filled_percent_() > object->GetPercentFilled()) {
          continue;
        }

        objectVec.push_back(object);
      }

      // Gets the contour that is bigger
      std::sort(objectVec.begin(), objectVec.end(), AreaSorts);

      // Eliminate same target
      if (eliminate_same_x_targets_()) {
        EliminateSameXTarget(objectVec);
      }

      // Choose red if need be
      if (detect_red_() && objectVec.size() > 1) {
        ChooseMostRed(objectVec);
      }

      // Since we search only one buoy, get the biggest from sort function
      if (objectVec.size() != 0) {
        Target buoy;
        std::stringstream message;
        buoy.SetTarget(objectVec[0], "buoy");
        NotifyTarget(buoy);

        if (debug_good_contour_()) {
          objectVec[0]->GetContourCopy().DrawContours(output_image_,
                                                      CV_RGB(0, 255, 0), 5);
        }
      }

      if (debug_good_contour_()) {
        output_image_.copyTo(image);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  float getRadiusFromRectangle(ObjectFullData::Ptr &rectangle);

  bool IsSameX(ObjectFullData::Ptr a, ObjectFullData::Ptr b);

  // check if ref is higher than compared
  bool IsHigher(ObjectFullData::Ptr ref, ObjectFullData::Ptr compared);

  void EliminateSameXTarget(ObjectFullData::FullObjectPtrVec &vec);

  void ChooseMostRed(ObjectFullData::FullObjectPtrVec &vec);

  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat output_image_;
  ObjectFeatureFactory feat_factory_;
  // Params
  Parameter<bool> enable_, debug_good_contour_, eliminate_same_x_targets_,
      detect_red_;
  Parameter<std::string> color_;
  RangedParameter<double> min_area_, max_ratio_, min_filled_percent_,
      max_x_difference_for_elimination_, ratio_for_angle_check_,
      admissible_horizontal_angle_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline float BuoySingle::getRadiusFromRectangle(
    ObjectFullData::Ptr &rectangle) {
  return (rectangle->GetRotatedRect().size.width / 2 +
          rectangle->GetRotatedRect().size.height / 2) /
         2;
}

//------------------------------------------------------------------------------
//
inline bool BuoySingle::IsSameX(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
  double x_difference = static_cast<double>(a->GetCenter().x) -
                        static_cast<double>(b->GetCenter().x);
  double abs_x_difference = fabs(x_difference);
  return abs_x_difference < max_x_difference_for_elimination_();
}

//------------------------------------------------------------------------------
//
// check if ref is higher than compared
inline bool BuoySingle::IsHigher(ObjectFullData::Ptr ref,
                                 ObjectFullData::Ptr compared) {
  return ref->GetCenter().y < compared->GetCenter().y;
}

//------------------------------------------------------------------------------
//
inline void BuoySingle::EliminateSameXTarget(
    ObjectFullData::FullObjectPtrVec &vec) {
  std::vector<unsigned int> index_to_eliminate;
  // We should not have much target, so double loop is ok...
  for (unsigned int i = 0, size = vec.size(); i < size; i++) {
    for (unsigned int j = 0; j < size; j++) {
      if (i == j) {
        continue;
      }
      if (IsSameX(vec[i], vec[j])) {
        // If I is higher, eliminate it
        if (IsHigher(vec[i], vec[j])) {
          index_to_eliminate.push_back(i);
        }
      }
    }
  }
  // Erase from vector
  if (index_to_eliminate.size() > 0) {
    for (int i = index_to_eliminate.size() - 1; i >= 0; i--) {
      vec.erase(vec.begin() + i);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void BuoySingle::ChooseMostRed(ObjectFullData::FullObjectPtrVec &vec) {
  ObjectFullData::Ptr a = vec[0], b = vec[1];
  cv::Mat original_img, hsv;
  cv::cvtColor(global_params_.getOriginalImage(), hsv, CV_BGR2HSV);
  cv::copyMakeBorder(hsv, hsv, 21, 21, 21, 21, cv::BORDER_CONSTANT);
  cv::Mat out;
  cv::cvtColor(a->GetBinaryImage(), out, CV_GRAY2BGR);
  cv::Point center_a = vec[0]->GetCenter(), center_b = vec[1]->GetCenter();

  cv::Mat roiA(
      hsv, cv::Rect(cv::Point(center_a.x - 5, (center_a.y)), cv::Size(20, 20)));
  cv::Mat roiB(
      hsv, cv::Rect(cv::Point(center_b.x - 5, (center_b.y)), cv::Size(20, 20)));

  cv::Scalar meanA = cv::mean(roiA);
  cv::Scalar meanB = cv::mean(roiB);

  // If a is more green, then it is yellow so we swap
  if (meanB[0] > meanA[0]) {
    std::swap(vec[0], vec[1]);
  }
}

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_BUOY_SINGLE_H_
