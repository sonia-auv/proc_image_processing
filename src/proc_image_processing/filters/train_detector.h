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

#ifndef PROVIDER_VISION_FILTERS_TRAIN_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_TRAIN_DETECTOR_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/algorithm/object_full_data.h>
#include <proc_image_processing/algorithm/performance_evaluator.h>
#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>

namespace proc_image_processing {

class TrainDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TrainDetector>;

  //==========================================================================
  // N E S T E D   C L A S S   D E F I N I T I O N

  class ObjectPair {
   public:
    //==========================================================================
    // P U B L I C   C / D T O R S

    explicit ObjectPair(ObjectFullData::Ptr object1,
                        ObjectFullData::Ptr object2,
                        ObjectFeatureFactory &featFactory)
        : _object1(object1), _object2(object2), _convexity_mean(0) {
      assert(object1.get() != nullptr);
      assert(object2.get() != nullptr);

      featFactory.ConvexityFeature(object1);
      featFactory.ConvexityFeature(object2);
      _convexity_mean = _object1->GetConvexity();
      _convexity_mean += _object2->GetConvexity();
      _convexity_mean /= 2.0f;
    }

    ~ObjectPair() {}

    static inline bool ConvexitySort(const ObjectPair &a, const ObjectPair &b) {
      return a._convexity_mean > b._convexity_mean;
    }

    ObjectFullData::Ptr _object1, _object2;
    float _convexity_mean;
  };

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TrainDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        pair_distance_maximum_("Pair_distance_maximum", 200, 0, 10000,
                               &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_),

        feat_factory_(3) {
    SetName("TrainDetector");
  }

  virtual ~TrainDetector() {}

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

        objVec.push_back(object);
      }

      std::vector<ObjectPair> pairs;
      // Iterate through object to find pair between objects
      for (int i = 0, size = objVec.size(); i < size; i++) {
        ObjectFullData::Ptr currentObj = objVec[i];
        if (currentObj.get() == nullptr) continue;
        for (int j = 0; j < size; j++) {
          if (j == i || objVec[j].get() == nullptr) continue;

          // If they are near enough, add it to the pair.
          float distance = EucledianPointDistance(objVec[j]->GetCenter(),
                                                  currentObj->GetCenter());
          if (distance <= pair_distance_maximum_()) {
            pairs.push_back(ObjectPair(currentObj, objVec[j], feat_factory_));
          }
        }
      }

      // Send the target
      if (pairs.size() != 0) {
        // Here we assume that the form founded will be almost perfect, so
        // convexity is 1.
        // We get the best pair and hope for the best
        std::sort(pairs.begin(), pairs.end(), ObjectPair::ConvexitySort);

        Target target;
        contour_t obj1(pairs[0]._object1->GetContourCopy().GetContour()),
            obj2(pairs[0]._object2->GetContourCopy().GetContour());

        for (int i = 0, size = obj2.size(); i < size; i++) {
          obj1.push_back(obj2[i]);
        }
        ObjectFullData::Ptr object =
            std::make_shared<ObjectFullData>(originalImage, image, obj1);
        cv::Point center = object->GetCenter();
        target.SetTarget("train", center.x, center.y, object->GetLength(),
                         object->GetLength(),
                         std::abs(object->GetRotatedRect().angle - 90),
                         image.rows, image.cols);
        NotifyTarget(target);
        if (debug_contour_()) {
          contourList_t tmp;
          tmp.push_back(obj1);
          cv::drawContours(output_image_, tmp, 0, CV_RGB(0, 255, 0), 2);
          cv::circle(output_image_, object->GetCenter(), 3, CV_RGB(0, 255, 0),
                     3);
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
  RangedParameter<int> pair_distance_maximum_, min_area_;

  ObjectFeatureFactory feat_factory_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_TRAIN_DETECTOR_H_
