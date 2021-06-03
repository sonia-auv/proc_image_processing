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

#ifndef PROVIDER_VISION_FILTERS_TRACK_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_TRACK_DETECTOR_H_

#include <algorithm/general_function.h>
#include <algorithm/object_feature_factory.h>
#include <algorithm/object_full_data.h>
#include <algorithm/type_and_const.h>
#include <filters/filter.h>
#include <server/target.h>
#include <memory>
#include <vector>

namespace proc_image_processing {

class TrackDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TrackDetector>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TrackDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_),
        targeted_ratio_("Ratio_target", 0.5f, 0.0f, 1.0f, &parameters_),
        difference_from_target_ratio_("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, &parameters_),
        feat_factory_(3) {
    SetName("TrackDetector");
  }

  virtual ~TrackDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (debug_contour_()) {
        image.copyTo(output_image_);
        if (output_image_.channels() == 1) {
          cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
        }
      }
      if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);

      cv::Mat originalImage = global_params_.getOriginalImage();
      contourList_t contours, inner_most_contour;
      hierachy_t hierachy;
      // Should optimize to find only one time and parse afterward...
      RetrieveOuterContours(image, contours);
      RetrieveInnerContours(image, inner_most_contour);
      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
        contour_t hull;
        cv::convexHull(contours[i], hull, false);
        ObjectFullData::Ptr object =
            std::make_shared<ObjectFullData>(originalImage, image, hull);
        if (object.get() == nullptr) {
          continue;
        }
        //
        // AREA
        //
        if (object->GetArea() < min_area_()) {
          continue;
        }
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
        }

        objVec.push_back(object);
      }

      std::sort(objVec.begin(), objVec.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                  return a->GetArea() > b->GetArea();
                });

      // Get all the square contours.
      contourList_t squareContour;
      for (auto innerContour : inner_most_contour) {
        if (cv::contourArea(innerContour) < min_area_()) {
          continue;
        }

        if (IsRectangle(innerContour, 10)) {
          squareContour.push_back(innerContour);
        }
      }
      if (debug_contour_()) {
        cv::drawContours(output_image_, squareContour, -1, CV_RGB(255, 0, 255),
                         3);
      }

      // Votes for the contour with the most
      std::vector<std::pair<ObjectFullData::Ptr, int>> contour_vote;
      for (auto &square : squareContour) {
        for (auto &already_voted_for : contour_vote) {
          if (cv::pointPolygonTest(
                  already_voted_for.first->GetContourCopy().GetContour(),
                  cv::Point2f(square[0].x, square[0].y), false) > 0.0f) {
            already_voted_for.second++;
            continue;
          }
        }

        for (auto &to_added_to_the_voted_pool : objVec) {
          if (cv::pointPolygonTest(
                  to_added_to_the_voted_pool->GetContourCopy().GetContour(),
                  cv::Point2f(square[0].x, square[0].y), false) > 0.0f) {
            contour_vote.push_back(std::pair<ObjectFullData::Ptr, int>(
                to_added_to_the_voted_pool, 1));
            cv::polylines(
                output_image_,
                to_added_to_the_voted_pool->GetContourCopy().GetContour(), true,
                CV_RGB(255, 0, 255), 3);
            continue;
          }
        }
      }
      std::sort(contour_vote.begin(), contour_vote.end(),
                [](const std::pair<ObjectFullData::Ptr, int> &a,
                   const std::pair<ObjectFullData::Ptr, int> &b) -> bool {
                  return a.second > b.second;
                });

      // Since we search only one buoy, get the biggest from sort function
      if (contour_vote.size() > 0) {
        Target target;
        ObjectFullData::Ptr object = contour_vote[0].first;
        cv::Point center = object->GetCenter();
        target.SetTarget("track", center.x, center.y, object->GetWidth(),
                         object->GetHeight(), object->GetRotatedRect().angle,
                         image.rows, image.cols);
        NotifyTarget(target);
        if (debug_contour_()) {
          cv::circle(output_image_, objVec[0]->GetCenter(), 3,
                     CV_RGB(0, 255, 0), 3);
        }
      }

      if (debug_contour_()) {
        output_image_.copyTo(image);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat output_image_;
  // Params
  Parameter<bool> enable_, debug_contour_;
  RangedParameter<double> min_area_, targeted_ratio_,
      difference_from_target_ratio_;

  ObjectFeatureFactory feat_factory_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_TRACK_DETECTOR_H_
