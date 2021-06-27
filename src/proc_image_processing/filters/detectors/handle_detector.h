/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_HANDLE_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_HANDLE_DETECTOR_H_

#include <proc_image_processing/algorithm/object_feature_factory.h>
#include <proc_image_processing/algorithm/object_full_data.h>
#include <proc_image_processing/algorithm/performance_evaluator.h>
#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>

namespace proc_image_processing {

  class HandleDetector : public AbstractFilter {
  public:
    using Ptr = std::shared_ptr<HandleDetector>;

    explicit HandleDetector(const GlobalParamHandler& globalParams)
      : AbstractFilter(globalParams),
      debug_contour_("Debug_contour", false, &parameters_),
      look_for_rectangle_("Look_for_Rectangle", false, &parameters_),
      disable_ratio_("disable_ratio_check", false, &parameters_),
      disable_angle_("disable_angle_check", false, &parameters_),
      id_("ID", "buoy", &parameters_),
      spec_1_("spec1", "red", &parameters_),
      spec_2_("spec2", "blue", &parameters_),
      min_area_("Min_area", 200, 0, 10000, &parameters_),
      targeted_ratio_("Ratio_target", 0.5f, 0.0f, 1.0f, &parameters_),
      difference_from_target_ratio_("Diff_from_ratio_target", 0.10f, 0.0f,
        1.0f, &parameters_),
      targeted_angle_("angle_target", 0.0f, 0.0f, 90.0f, &parameters_),
      difference_from_target_angle_("Diff_from_angle_target", 30.0f, 0.0f,
        90.0f, &parameters_),
      feature_factory_(5) {
      SetName("HandleDetector");
      // Little goodies for cvs
      // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
    }

    virtual ~HandleDetector() {}

    virtual void ProcessImage(cv::Mat& image) {

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

          // AREA
          if (object->GetArea() < min_area_()) {
            continue;
          }
          if (debug_contour_()) {
            cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
          }

          // RATIO
          feature_factory_.RatioFeature(object);
          if (!disable_ratio_() && (fabs(object->GetRatio() - targeted_ratio_()) >
            fabs(difference_from_target_ratio_()))) {
            continue;
          }
          if (debug_contour_()) {
            cv::drawContours(output_image_, contours, i, CV_RGB(0, 0, 255), 2);
          }

          // ANGLE
          if (!disable_angle_() &&
            (fabs(object->GetRotatedRect().angle - targeted_angle_()) >
              fabs(difference_from_target_angle_()))) {
            continue;
          }

          // RECTANGLE
          if (look_for_rectangle_() && !IsRectangle(contours[i], 10)) {
            continue;
          }

          if (debug_contour_()) {
            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
          }

          objVec.push_back(object);
        }

        std::sort(objVec.begin(), objVec.end(),
          [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
            return a->GetArea() > b->GetArea();
          });

        // Since we search only one buoy, get the biggest from sort function
        if (objVec.size() > 0) {
          Target target;
          ObjectFullData::Ptr object = objVec[0];
          cv::Point center = object->GetCenter();
          target.SetTarget(id_(), center.x, center.y, object->GetWidth(),
            object->GetHeight(), object->GetRotatedRect().angle,
            image.rows, image.cols);
          target.SetSpecField_1(spec_1_());
          target.SetSpecField_2(spec_2_());
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

  private:
    cv::Mat output_image_;
    // Params
    Parameter<bool> debug_contour_, look_for_rectangle_, disable_ratio_,
      disable_angle_;
    Parameter<std::string> id_, spec_1_, spec_2_;
    RangedParameter<double> min_area_, targeted_ratio_,
      difference_from_target_ratio_, targeted_angle_,
      difference_from_target_angle_;

    ObjectFeatureFactory feature_factory_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_OBJECT_FINDER_H_
