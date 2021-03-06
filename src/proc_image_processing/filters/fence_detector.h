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

#ifndef PROVIDER_VISION_FILTERS_FENCE_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_FENCE_DETECTOR_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/algorithm/object_feature_factory.h>
#include <proc_image_processing/algorithm/object_full_data.h>
#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>

namespace proc_image_processing {

class FenceDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FenceDetector>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit FenceDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        search_only_bottom_("Search_only_bottom", false, &parameters_,
                            "Enables searching only for bottom bar"),
        min_length_("Minimum_length", 50, 0, 2000, &parameters_),
        max_distance_from_bottom_bar_extremum_("Max_dist_from_extremum", 50, 0,
                                               2000, &parameters_),
        min_area_("Minimum_area", 300, 0, 10000, &parameters_),
        max_diff_from_90_tbca_horizontal_(
            "Max_diff_horizontal", 20, 0, 90, &parameters_,
            "Maximum difference from 90 to be consider as horizontal"),
        max_diff_from_0_tbca_vertical_(
            "Max_diff_vertical", 20, 0, 90, &parameters_,
            "Maximum difference from 0 to be consider as vertical"),
        min_percent_filled_("Minimum_percent_filled", 70, 0, 1000,
                            &parameters_),
        feat_factory_(3) {
    SetName("FenceDetector");
  }

  virtual ~FenceDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (!enable_()) {
      return;
    }

    cv::Mat in;
    if (debug_contour_()) {
      // Case we receive a color or gray scale image.
      if (image.channels() == 1) {
        cv::cvtColor(image, output_image_, CV_GRAY2BGR);
      } else {
        image.copyTo(output_image_);
      }
    }

    if (image.channels() != 1) {
      cv::cvtColor(image, in, CV_BGR2GRAY);
    } else {
      image.copyTo(in);
    }

    contourList_t contours;
    RetrieveOuterContours(in, contours);
    std::vector<ObjectFullData::Ptr> verticalBars, horizontalBar,
        merged_horizontal_bar;

    cv::Mat originalImage = global_params_.getOriginalImage();

    // Parse contours into 2 categories, vertical or horizontal.
    for (int i = 0, size = contours.size(); i < size; i++) {
      ObjectFullData::Ptr object =
          std::make_shared<ObjectFullData>(originalImage, image, contours[i]);

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

      //
      // LENGTH
      //
      if (object->GetRotatedRect().size.height < min_length_()) continue;
      if (debug_contour_()) {
        cv::drawContours(output_image_, contours, i, CV_RGB(255, 255, 0), 3);
      }
      feat_factory_.PercentFilledFeature(object);

      if (int(object->GetPercentFilled() * 100.0f) < min_percent_filled_())
        continue;
      if (debug_contour_()) {
        cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 255), 4);
      }

      float angle = fabs(object->GetRotatedRect().angle);

      if (angle < max_diff_from_0_tbca_vertical_()) {
        verticalBars.push_back(object);
      } else if ((90 - angle) < max_diff_from_90_tbca_horizontal_()) {
        horizontalBar.push_back(object);
      }
    }

    // Sort the bars with different criteria
    // Here we look for horizontal bar because it's
    std::sort(horizontalBar.begin(), horizontalBar.end(),
              [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                return a->GetRotatedRect().size.height >
                       b->GetRotatedRect().size.height;
              });

    if (horizontalBar.size() >= 2) {
      std::vector<std::pair<int, int>> pairs;
      for (int ref_idx = 0, size_ref = horizontalBar.size(); ref_idx < size_ref;
           ref_idx++) {
        if (IsSplitBar(horizontalBar[0], horizontalBar[1])) {
          contour_t tmp, a, b;
          a = horizontalBar[0]->GetContourCopy().GetContour();
          b = horizontalBar[1]->GetContourCopy().GetContour();

          tmp.reserve(a.size() + b.size());
          tmp.insert(tmp.end(), a.begin(), a.end());
          tmp.insert(tmp.end(), b.begin(), b.end());
          horizontalBar[0] =
              std::make_shared<ObjectFullData>(originalImage, image, tmp);
        }
      }
    }

    //
    // the easiest to find... if you did not find this one...
    // you probably didn't find any other... by experience.
    // Also, you need to return a size to AUV6 so...
    if (horizontalBar.size() != 0) {
      // Gets bottom bar info.
      ObjectFullData::Ptr final_horizontal_bar = horizontalBar[0];
      RotRect rect_from_hori_bar = final_horizontal_bar->GetRotatedRect();
      if (debug_contour_()) {
        cv::circle(output_image_, rect_from_hori_bar.center, 3,
                   CV_RGB(0, 0, 255), 3);
      }

      Target fence;
      cv::Point center = (rect_from_hori_bar.center);
      fence.SetTarget("fence", center.x, center.y,
                      rect_from_hori_bar.size.width,
                      rect_from_hori_bar.size.height, rect_from_hori_bar.angle,
                      image.rows, image.cols);

      int y_coord_from_bottom = CalculateYFromBottomBar(
          rect_from_hori_bar.size.height, rect_from_hori_bar.center.y);

      if (search_only_bottom_()) {
        center = cv::Point(rect_from_hori_bar.center.x, y_coord_from_bottom);
        if (debug_contour_()) {
          cv::circle(output_image_, center, 5, CV_RGB(0, 255, 255), 20);
        }
        SetCameraOffset(center, image.rows, image.cols);

        fence.SetCenter(center);

      } else  // Gets the two best vertical bar to compute our y center.
      {
        std::sort(verticalBars.begin(), verticalBars.end(),
                  [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                    return a->GetRotatedRect().size.height >
                           b->GetRotatedRect().size.height;
                  });

        std::vector<cv::Point2f> final_vert_bar;
        int leftX, rightX;
        GetBottomBarXExtremum(final_horizontal_bar, leftX, rightX);

        // Extract the two major bar that are near the extremum of the bottom
        // post.
        int i = 0, size = verticalBars.size(), bar_founded = 0;
        for (; i < size && bar_founded != 2; i++) {
          if (IsNearExtremum(verticalBars[i]->GetRotatedRect().center.x, leftX,
                             rightX)) {
            final_vert_bar.push_back(verticalBars[i]->GetRotatedRect().center);
            if (debug_contour_()) {
              cv::circle(output_image_,
                         verticalBars[i]->GetRotatedRect().center, 5,
                         CV_RGB(0, 255, 255), 20);
            }
            bar_founded++;
          }
        }

        if (bar_founded == 2) {
          int x = 0, y = 0;
          // X from bottom bar plus
          x = (rect_from_hori_bar.center.x +
               (final_vert_bar[0].x + final_vert_bar[1].x) / 2) /
              2;

          y = (y_coord_from_bottom + final_vert_bar[0].y +
               final_vert_bar[1].y) /
              3;
          center = cv::Point(x, y);
          if (debug_contour_()) {
            cv::circle(output_image_, center, 5, CV_RGB(0, 255, 255), 20);
          }
          SetCameraOffset(center, image.rows, image.cols);
          fence.SetCenter(center);
        } else if (bar_founded == 1) {
          int y = (y_coord_from_bottom + final_vert_bar[0].y) / 2;
          center = cv::Point(rect_from_hori_bar.center.x, y);
          if (debug_contour_()) {
            cv::circle(output_image_, center, 5, CV_RGB(0, 255, 255), 20);
          }
          SetCameraOffset(center, image.rows, image.cols);
          fence.SetCenter(center);
        } else {
          center = cv::Point(rect_from_hori_bar.center.x, y_coord_from_bottom);
          if (debug_contour_()) {
            cv::circle(output_image_, center, 5, CV_RGB(0, 255, 255), 20);
          }
          SetCameraOffset(center, image.rows, image.cols);
          fence.SetCenter(center);
        }
      }
      NotifyTarget(fence);
    }
    if (debug_contour_()) {
      output_image_.copyTo(image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  inline int CalculateYFromBottomBar(float bar_size, int bar_y) {
    int offset = static_cast<int>((bar_size) / 5.0f);
    return bar_y - offset;
  }

  inline void GetBottomBarXExtremum(ObjectFullData::Ptr bottom_bar, int &leftX,
                                    int &rightX) {
    leftX = 20000;
    rightX = -1;
    contour_t contour = bottom_bar->GetContourCopy().GetContour();
    for (auto pt : contour) {
      if (leftX > pt.x) {
        leftX = pt.x;
      }
      if (rightX < pt.x) {
        rightX = pt.x;
      }
    }
  }

  inline bool IsNearExtremum(int x_coord, int leftX, int rightX) {
    return IsBetweenLimit(x_coord, leftX) || IsBetweenLimit(x_coord, rightX);
  }

  inline bool IsBetweenLimit(int pt_x, int ref_x) {
    int left_max = (ref_x - max_distance_from_bottom_bar_extremum_());
    int right_max = (ref_x + max_distance_from_bottom_bar_extremum_());
    bool left_ok = left_max < pt_x;
    bool right_ok = right_max > pt_x;
    return left_ok && right_ok;
  }

  inline bool IsSplitBar(ObjectFullData::Ptr ref, ObjectFullData::Ptr &comp) {
    float ratio_diff =
        std::abs(comp->GetRatio() - ref->GetRatio()) / ref->GetRatio();
    float y_diff =
        std::abs(comp->GetCenter().y - ref->GetCenter().y) / ref->GetCenter().y;

    bool ratio_ok = ratio_diff < 0.1;
    bool y_diff_ok = y_diff < 0.1;
    return ratio_ok && y_diff_ok;
  }

  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_, debug_contour_, search_only_bottom_;
  // tbca = To Be Consider As
  RangedParameter<int> min_length_, max_distance_from_bottom_bar_extremum_,
      min_area_, max_diff_from_90_tbca_horizontal_,
      max_diff_from_0_tbca_vertical_, min_percent_filled_;

  cv::Mat output_image_;
  ObjectFeatureFactory feat_factory_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_FENCE_DETECTOR_H_
