// FACTORY_GENERATOR_CLASS_NAME=HandleDetector

#ifndef PROC_IMAGE_PROCESSING_FILTERS_HANDLE_DETECTOR_H_
#define PROC_IMAGE_PROCESSING_FILTERS_HANDLE_DETECTOR_H_

#include <proc_image_processing/cpu/algorithm/object_feature_factory.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/server/target.h>
#include <memory>

namespace proc_image_processing {

    class HandleDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<HandleDetector>;

        explicit HandleDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  look_for_rectangle_("Look for rectangle", false, &parameters_),
                  disable_ratio_("Disable ratio check", false, &parameters_),
                  disable_angle_("Disable angle check", false, &parameters_),
                  id_("ID", "buoy", &parameters_),
                  spec_1_("spec1", "red", &parameters_),
                  spec_2_("spec2", "blue", &parameters_),
                  min_area_("Minimum area", 200, 0, 10000, &parameters_),
                  targeted_ratio_("Ratio target", 0.5f, 0.0f, 1.0f, &parameters_),
                  difference_from_target_ratio_("Difference from ratio target", 0.10f, 0.0f, 1.0f, &parameters_),
                  targeted_angle_("angle_target", 0.0f, 0.0f, 90.0f, &parameters_),
                  difference_from_target_angle_("Difference from angle target", 30.0f, 0.0f, 90.0f, &parameters_),
                  feature_factory_(5) {
            setName("HandleDetector");
            // Little goodies for cvs
            // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
        }

        ~HandleDetector() override = default;

        void apply(cv::Mat &image) override {
            if (debug_contour_()) {
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }
            }

            if (image.channels() != 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }

            cv::Mat originalImage = global_param_handler_.getOriginalImage();

            PerformanceEvaluator timer;
            timer.resetStartTime();

            contourList_t contours;
            retrieveAllContours(image, contours);
            ObjectFullData::FullObjectPtrVec objVec;
            for (int i = 0; i < contours.size(); i++) {
                ObjectFullData::Ptr object =
                        std::make_shared<ObjectFullData>(
                                output_image_,
                                image,
                                reinterpret_cast<Contour &&>(contours[i])
                        );;
                if (object.get() == nullptr) {
                    continue;
                }

                // AREA
                if (object->getArea() < min_area_()) {
                    continue;
                }
                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                }

                // RATIO
                feature_factory_.ratioFeature(object);
                if (!disable_ratio_() && (fabs(object->getRatio() - targeted_ratio_()) >
                                          fabs(difference_from_target_ratio_()))) {
                    continue;
                }
                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(0, 0, 255), 2);
                }

                // ANGLE
                if (!disable_angle_() &&
                    (fabs(object->getRotRect().angle - targeted_angle_()) >
                     fabs(difference_from_target_angle_()))) {
                    continue;
                }

                // RECTANGLE
                if (look_for_rectangle_() && !isRectangle(contours[i], 10)) {
                    continue;
                }

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                }

                objVec.push_back(object);
            }

            std::sort(objVec.begin(), objVec.end(),
                      [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                          return a->getArea() > b->getArea();
                      });

            // Since we search only one buoy, get the biggest from sort function
            if (!objVec.empty()) {
                Target target;
                ObjectFullData::Ptr object = objVec[0];
                cv::Point center = object->getCenterPoint();
                target.setTarget(id_(), center.x, center.y, object->getWidth(),
                                 object->getHeight(), object->getRotRect().angle,
                                 image.rows, image.cols);
                target.setSpecialField1(spec_1_());
                target.setSpecialField2(spec_2_());
                notify(target);
                if (debug_contour_()) {
                    cv::circle(output_image_, objVec[0]->getCenterPoint(), 3,
                               CV_RGB(0, 255, 0), 3);
                }
            }
            if (debug_contour_()) {
                output_image_.copyTo(image);
            }
        }

    private:
        cv::Mat output_image_;

        Parameter<bool> debug_contour_;
        Parameter<bool> look_for_rectangle_;
        Parameter<bool> disable_ratio_;
        Parameter<bool> disable_angle_;
        Parameter <std::string> id_;
        Parameter <std::string> spec_1_;
        Parameter <std::string> spec_2_;

        RangedParameter<double> min_area_;
        RangedParameter<double> targeted_ratio_;
        RangedParameter<double> difference_from_target_ratio_;
        RangedParameter<double> targeted_angle_;
        RangedParameter<double> difference_from_target_angle_;

        ObjectFeatureFactory feature_factory_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_HANDLE_DETECTOR_H_
