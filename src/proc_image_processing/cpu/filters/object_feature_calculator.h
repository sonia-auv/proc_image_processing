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

#ifndef PROVIDER_VISION_FILTERS_OBJECT_CALCULATOR_H_
#define PROVIDER_VISION_FILTERS_OBJECT_CALCULATOR_H_

#include <algorithm/general_function.h>
#include <algorithm/object_feature_factory.h>
#include <algorithm/object_full_data.h>
#include <algorithm/performance_evaluator.h>
#include <filters/filter.h>
#include <server/target.h>
#include <memory>
#include <string>

namespace proc_image_processing {

class ObjectFeatureCalculator : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectFeatureCalculator>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ObjectFeatureCalculator(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        toggle_recording_("toggle_recording", false, &parameters_),
        id_("ID", "buoy", &parameters_),
        spec_1_("spec1", "red", &parameters_),
        spec_2_("spec2", "blue", &parameters_),
        output_folder_("output_folder", "/home/jeremie/aidata/rec1/",
                       &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_),
        feature_factory_(5) {
    SetName("ObjectFeatureCalculator");
    //    feature_factory_.SetAllFeatureToCompute();
    // Little goodies for cvs
    // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
  }

  virtual ~ObjectFeatureCalculator() {}

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

      PerformanceEvaluator timer;
      timer.UpdateStartTime();

      contourList_t contours;
      RetrieveAllContours(image, contours);
      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
        ObjectFullData::Ptr object =
            std::make_shared<ObjectFullData>(originalImage, image, contours[i]);
        if (object.get() == nullptr) {
          continue;
        }

        if (object->GetArea() < min_area_()) {
          continue;
        }
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
        }

        if (IsRectangle(contours[i], 10)) {
          cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);

          objVec.push_back(object);
        }
      }
      // Calculate features
      //			feature_factory_.CalculateFeatureVectors(objVec);
      //			printf("Image parsing took: %f seconds\n",
      // timer.GetExecTime());
      //      if (_toggle_recording()) {
      //        AITrainer::OutputFrameData(_output_folder(), objVec,
      //        originalImage,
      //                                   image, _recording_frame_index);
      //        _recording_frame_index++;
      //      }
      if (debug_contour_()) {
        output_image_.copyTo(image);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat output_image_;

  Parameter<bool> enable_, debug_contour_, toggle_recording_;
  Parameter<std::string> id_, spec_1_, spec_2_, output_folder_;
  RangedParameter<double> min_area_;

  ObjectFeatureFactory feature_factory_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_OBJECT_CALCULATOR_H_
