/// TODO: Refactor code repetition

// FACTORY_GENERATOR_CLASS_NAME=EllipseDetector

#ifndef PROC_IMAGE_PROCESSING_ELLIPSE_DETECTOR_H
#define PROC_IMAGE_PROCESSING_ELLIPSE_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <cmath>
#include <memory>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>

namespace proc_image_processing {

    class EllipseDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<EllipseDetector>;

        explicit EllipseDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  look_for_ellipse_("Look for ellipse", false, &parameters_),
                  look_for_heart_("Look_for_Heart", false, &parameters_),
                  min_area_("Minimum area", 5000, 1, 50000, &parameters_),
                  max_area_("Maximum area", 100000, 1, 1000000, &parameters_),
                  percent_filled_("Percent filled", 0, 0, 100, &parameters_),
                  circle_index_("Circle index", 0, 0, 1, &parameters_),
                  obstacle_("Target name", "", &parameters_),
                  desc1_("Descriptor 1", "", &parameters_),
                  desc2_("Descriptor 2", "", &parameters_) {
            setName("EllipseDetector");
        }

        ~EllipseDetector() override = default;

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
                ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(
                        output_image_,
                        image,
                        reinterpret_cast<Contour &&>(contours[i])
                );
                if (object.get() == nullptr) {
                    continue;
                }

                if (object->getArea() < min_area_() || object->getArea() > max_area_()) {
                    continue;
                }

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                }

                // At least 5 points are required
                if (contours[i].size() >= 5) {
                    cv::Mat pointfs;
                    cv::Mat(contours[i]).convertTo(pointfs, CV_32F);
                    cv::RotatedRect box = cv::fitEllipse(pointfs);

                    float circleIndex;
                    float percentFilled;

                    if (look_for_ellipse_()) {
                        circleIndex = getCircleIndex(contours[i]);

                        if (circleIndex < circle_index_()) {
                            continue;
                        }

                        percentFilled = getPercentFilled(output_image_, box);

                        if (percentFilled > percent_filled_()) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objective = obstacle_();
                    }

                    if (look_for_heart_()) {
                        circleIndex = getCircleIndex(contours[i]);

                        if (circleIndex > circle_index_()) {
                            continue;
                        }

                        percentFilled = getPercentFilled(output_image_, box);

                        if (percentFilled > percent_filled_()) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objective = obstacle_();
                    }
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
        Parameter<bool> look_for_ellipse_;
        Parameter<bool> look_for_heart_;

        Parameter<std::string> obstacle_;
        Parameter<std::string> desc1_;
        Parameter<std::string> desc2_;

        RangedParameter<double> min_area_;
        RangedParameter<double> max_area_;
        RangedParameter<double> circle_index_;

        RangedParameter<int> percent_filled_;

    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ELLIPSE_DETECTOR_H
