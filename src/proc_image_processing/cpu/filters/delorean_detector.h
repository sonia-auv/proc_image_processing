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

#ifndef PROVIDER_VISION_FILTERS_DELOREAN_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_DELOREAN_DETECTOR_H_

#include <algorithm/general_function.h>
#include <algorithm/object_feature_factory.h>
#include <algorithm/object_full_data.h>
#include <filters/filter.h>
#include <server/target.h>
#include <memory>

namespace proc_image_processing {

class DeloreanDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DeloreanDetector>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit DeloreanDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        output_train_("Output_train", false, &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_),
        targeted_ratio_big_("Ratio_target_big", 0.5f, 0.0f, 1.0f, &parameters_),
        targeted_ratio_small_("Ratio_target_small", 0.5f, 0.0f, 1.0f,
                              &parameters_),
        difference_from_target_ratio_("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, &parameters_),
        feat_factory_(3) {
    SetName("DeloreanDetector");
  }

  virtual ~DeloreanDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() != 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }

      if (debug_contour_()) {
        image.copyTo(output_image_);
        if (output_image_.channels() == 1) {
          cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
        }
      }

      cv::Mat originalImage = global_params_.getOriginalImage();
      contourList_t contours;
      hierachy_t hierachy;
      RetrieveOuterContours(image, contours);
      ObjectFullData::FullObjectPtrVec objVec_big;
      ObjectFullData::FullObjectPtrVec objVec_small;
      for (size_t i = 0, size = contours.size(); i < size; i++) {
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
          cv::drawContours(output_image_, contours, int(i), CV_RGB(255, 0, 0),
                           2);
        }

        //
        // RATIO
        //
        //        std::stringstream ss;
        //        ss << object->GetRatio();
        //        cv::putText(output_image_, ss.str(), object->GetCenter(),
        //                    cv::FONT_HERSHEY_SIMPLEX, 1 /*fontscale*/,
        //                    cv::Scalar(255, 0, 0), 3 /*thickness*/, 6
        //                    /*lineType*/,
        //                    false);
        feat_factory_.RatioFeature(object);
        double ratio_difference_big =
            fabs(object->GetRatio() - targeted_ratio_big_());
        double ratio_difference_small =
            fabs(object->GetRatio() - targeted_ratio_small_());
        //				std::cout << object->GetRatio() << " "
        //<<
        // targeted_ratio_() << " " << ratio << std::endl;

        // Could change for check for wich is nearest the target ratio, i.e. is
        // the ratio nearrer big object or
        // small object...
        if (ratio_difference_big > difference_from_target_ratio_() &&
            ratio_difference_small < difference_from_target_ratio_()) {
          objVec_small.push_back(object);
        } else if (ratio_difference_big < difference_from_target_ratio_() &&
                   ratio_difference_small > difference_from_target_ratio_()) {
          objVec_big.push_back(object);
        } else if (ratio_difference_big < difference_from_target_ratio_() &&
                   ratio_difference_small < difference_from_target_ratio_()) {
          objVec_big.push_back(object);

        } else {
          continue;
        }
        //				if( !IsRectangle( contours[i] ) )
        //				{
        //					continue;
        //				}
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, int(i), CV_RGB(0, 0, 255),
                           2);
        }
      }

      std::sort(objVec_big.begin(), objVec_big.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                  return a->GetArea() > b->GetArea();
                });
      std::sort(objVec_small.begin(), objVec_small.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                  return a->GetArea() > b->GetArea();
                });

      if (objVec_big.size() > 0) {
        Target target;
        ObjectFullData::Ptr object_big = objVec_big[0];
        cv::Point center_big = object_big->GetCenter();
        double angle = 181;
        if (objVec_small.size() > 0) {
          ObjectFullData::Ptr object_small = objVec_small[0];
          cv::Point center_small = object_small->GetCenter();
          if (debug_contour_()) {
            cv::circle(output_image_, center_small, 2, CV_RGB(255, 0, 255), 2);
          }
          angle = 360 * (atan2(center_big.y - center_big.y,
                               center_big.x + 5 - center_big.x) -
                         atan2(center_small.y - center_big.y,
                               center_small.x - center_big.x)) /
                  (2 * 3.1416);
        }

        if (output_train_()) {
          target.SetTarget("train", center_big.x, center_big.y,
                           object_big->GetHeight(), object_big->GetHeight(),
                           float(angle), image.rows, image.cols);
        } else {
          target.SetTarget("delorean", center_big.x, center_big.y,
                           object_big->GetHeight(), object_big->GetHeight(),
                           float(angle), image.rows, image.cols);
        }
        NotifyTarget(target);
        if (debug_contour_()) {
          cv::circle(output_image_, objVec_big[0]->GetCenter(), 3,
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

  Parameter<bool> enable_, debug_contour_, output_train_;

  RangedParameter<double> min_area_, targeted_ratio_big_, targeted_ratio_small_,
      difference_from_target_ratio_;

  ObjectFeatureFactory feat_factory_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_DELOREAN_DETECTOR_H_
