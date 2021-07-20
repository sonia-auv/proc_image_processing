// FACTORY_GENERATOR_CLASS_NAME=CenterCoffinDetector

#ifndef PROC_IMAGE_PROCESSING_CENTER_COFFIN_DETECTOR_H
#define PROC_IMAGE_PROCESSING_CENTER_COFFIN_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <cmath>
#include <memory>

namespace proc_image_processing {

    class CenterCoffinDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<CenterCoffinDetector>;

        explicit CenterCoffinDetector(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  debug_contour_("Debug_contour", false, &parameters_),
                  look_for_rectangle_("Look_for_Rectangle", false, &parameters_),
                  min_area_("Min_area", 100, 1, 10000, &parameters_),
                  max_area_("Max_area", 1000, 1, 1000000, &parameters_) {
            setName("CenterCoffinDetector");
        }

        ~CenterCoffinDetector() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                std::string objective;
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);

                PerformanceEvaluator timer;
                timer.resetStartTime();

                contourList_t contours;

                retrieveAllContours(image, contours);
                ObjectFullData::FullObjectPtrVec objVec;
                for (int i = 0; i < contours.size(); i++) {
                    ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image, contours[i]);
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

                        objective = "surface";

                    }

                    objVec.push_back(object);
                }

                std::sort(objVec.begin(), objVec.end(),
                          [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                              return a->getArea() >
                                     b->getArea();
                          });

                if (objVec.size() > 1) {
                    Target target;
                    ObjectFullData::Ptr object_1 = objVec[0];
                    ObjectFullData::Ptr object_2 = objVec[1];

                    cv::Point center_1 = object_1->getCenterPoint();
                    cv::Point center_2 = object_2->getCenterPoint();

                    cv::Point target_center;
                    target_center.x = (center_1.x + center_2.x) / 2;
                    target_center.y = (center_1.y + center_2.y) / 2;

                    target.setTarget(objective, target_center.x, target_center.y, object_1->getWidth(),
                                     object_1->getHeight(), object_1->getRotRect().angle, image.rows, image.cols);
                    notify(target);
                    if (debug_contour_()) {
                        cv::circle(output_image_, target_center, 100, CV_RGB(0, 255, 0), 3);
                    }
                }

                if (debug_contour_()) {
                    output_image_.copyTo(image);
                }
            }
        }

    private:
        cv::Mat output_image_;

        Parameter<bool> enable_, debug_contour_, look_for_rectangle_;

        RangedParameter<double> min_area_, max_area_;
    };

}  // namespace proc_image_processing

#endif  //PROC_IMAGE_PROCESSING_CENTER_COFFIN_DETECTOR_H
