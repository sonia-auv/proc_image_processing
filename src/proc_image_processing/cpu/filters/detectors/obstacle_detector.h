// FACTORY_GENERATOR_CLASS_NAME=ObstacleDetector

#ifndef PROC_IMAGE_PROCESSING_OBSTACLE_BODY_DETECTOR_H
#define PROC_IMAGE_PROCESSING_OBSTACLE_BODY_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <cmath>
#include <memory>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>

namespace proc_image_processing {

    class ObstacleDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<ObstacleDetector>;

        explicit ObstacleDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  look_for_rectangle_("Look for rectangle", false, &parameters_),
                  min_area_("Minimum area", 100, 1, 10000, &parameters_),
                  max_area_("Maximum area", 1000, 1, 50000, &parameters_) {
            setName("ObstacleDetector");
        }

        ~ObstacleDetector() override = default;

        void apply(cv::Mat &image) override {
            std::string objective;
            image.copyTo(output_image_);
            if (output_image_.channels() == 1) {
                cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
            }

            if (image.channels() != 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }

            PerformanceEvaluator timer;
            timer.resetStartTime();

            contourList_t contours;

            retrieveAllContours(image, contours);
            ObjectFullData::FullObjectPtrVec objVec;
            for (int i = 0; i < contours.size(); i++) {
                ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image,
                                                                              reinterpret_cast<Contour &&>(contours[i]));
                if (object.get() == nullptr) {
                    continue;
                }

                if (object->getArea() < min_area_() || object->getArea() > max_area_()) {
                    continue;
                }

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                }

                if (look_for_rectangle_() && isRectangle(contours[i], 20)) {
                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                    }
                    objective = "obstacle";
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

            if (!objVec.empty()) {
                Target target;
                ObjectFullData::Ptr object = objVec[0];
                cv::Point center = object->getCenterPoint();
                target.setTarget(
                        objective,
                        center.x,
                        center.y,
                        object->getWidth(),
                        object->getHeight(),
                        object->getRotRect().angle,
                        image.rows,
                        image.cols
                );
                notify(target);
                if (debug_contour_()) {
                    cv::circle(output_image_, objVec[0]->getCenterPoint(), 3, CV_RGB(0, 255, 0), 3);
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
        RangedParameter<double> min_area_;
        RangedParameter<double> max_area_;
    };

}  // namespace proc_image_processing

#endif  //PROC_IMAGE_PROCESSING_OBSTACLE_DETECTOR_H
