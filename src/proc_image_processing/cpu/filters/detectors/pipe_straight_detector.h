// FACTORY_GENERATOR_CLASS_NAME=PipeStraightDetector

#ifndef PROC_IMAGE_PROCESSING_FILTERS_PIPE_STRAIGHT_DETECTOR_H_
#define PROC_IMAGE_PROCESSING_FILTERS_PIPE_STRAIGHT_DETECTOR_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <proc_image_processing/cpu/algorithm/line.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/server/target.h>
#include <memory>

namespace proc_image_processing {

    class PipeStraightDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<PipeStraightDetector>;

        explicit PipeStraightDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  min_area_("Minimum area", 200, 0, 10000, &parameters_, "Min area"),
                  min_pixel_("Minimum pixel", 0, 20, 100, &parameters_, "Min pixel") {
            setName("PipeStraightDetector");
        }

        ~PipeStraightDetector() override = default;

        void apply(cv::Mat &image) override {
            image.copyTo(output_image_);
            if (debug_contour_()) {
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
            retrieveOuterContours(image, contours);
            ObjectFullData::FullObjectPtrVec objVec;
            for (int i = 0; i < contours.size(); i++) {
                ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(
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

                Line lineFit = getLineOnPolygon(contours[i], output_image_.cols);

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                }
                objVec.push_back(object);
            }

            std::sort(
                    objVec.begin(),
                    objVec.end(),
                    [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                        return a->getArea() > b->getArea();
                    }
            );

            // Since we search only one buoy, get the biggest from sort function
            if (!objVec.empty()) {
                Target target;
                ObjectFullData::Ptr object = objVec[0];
                cv::Point center = object->getCenterPoint();
                target.setTarget("pipe straight",
                                 center.x,
                                 center.y,
                                 object->getWidth(),
                                 object->getHeight(),
                                 object->getAngle(),
                                 image.rows,
                                 image.cols
                );
                notify(target);
                if (debug_contour_()) {
                    cv::circle(
                            output_image_,
                            objVec[0]->getCenterPoint(),
                            3,
                            CV_RGB(0, 255, 0),
                            3
                    );
                }
            }
            if (debug_contour_()) {
                output_image_.copyTo(image);
            }
        }

    private:
        Parameter<bool> debug_contour_;
        RangedParameter<double> min_area_;
        RangedParameter<double> min_pixel_;

        cv::Mat output_image_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_PIPE_STRAIGHT_DETECTOR_H_