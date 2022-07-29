// FACTORY_GENERATOR_CLASS_NAME=ContourDetector

#ifndef PROC_IMAGE_PROCESSING_CONTOUR_DETECTOR_H
#define PROC_IMAGE_PROCESSING_CONTOUR_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <cmath>
#include <memory>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>

//Origin = obstacle_detector
// Look at the contours and point to the Nth one


namespace proc_image_processing {

    class ContourDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<ContourDetector>;

        explicit ContourDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  objective_("Target name", "", &parameters_),
                  wanted_index_("N biggest contour", 1,1,10, &parameters_, "N contour"),
                  min_area_("Minimum area", 100, 1, 10000, &parameters_, "Min area"),
                  max_area_("Maximum area", 5000, 1, 100000, &parameters_, "Max area") {
            setName("ContourDetector");
        }

        ~ContourDetector() override = default;

        void apply(cv::Mat &image) override {
            std::string objective;
            std::string desc1 = "";
            std::string desc2 = "";
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
                if (object.get() == nullptr || object->getArea() < min_area_() || object->getArea() > max_area_()) {
                    continue;
                }

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                    objective = objective_();
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
                int index = (wanted_index_() >= objVec.size())? objVec.size() -1 : wanted_index_();
                ObjectFullData::Ptr object = objVec[index];
                cv::Point center = object->getCenterPoint();

                target.setTarget(
                        objective,
                        center.x,
                        center.y,
                        object->getWidth(),
                        object->getHeight(),
                        object->getRotRect().angle,
                        image.rows,
                        image.cols,
                        desc1,
                        desc2
                );

                notify(target);
                if (debug_contour_()) {
                    cv::circle(output_image_, objVec[index]->getCenterPoint(), 3, CV_RGB(0, 255, 0), 3);
                }
            }

            if (debug_contour_()) {
                output_image_.copyTo(image);
            }
        }

    private:
        cv::Mat output_image_;
        Parameter<bool> debug_contour_;
        Parameter<std::string> objective_;
        RangedParameter<int> wanted_index_;
        RangedParameter<double> min_area_;
        RangedParameter<double> max_area_;
    };

}  // namespace proc_image_processing

#endif  //PROC_IMAGE_PROCESSING_CONTOUR_DETECTOR_H
