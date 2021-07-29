// FACTORY_GENERATOR_CLASS_NAME=VampireBodyDetector

#ifndef PROC_IMAGE_PROCESSING_VAMPIRE_BODY_DETECTOR_H
#define PROC_IMAGE_PROCESSING_VAMPIRE_BODY_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <cmath>
#include <memory>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>

namespace proc_image_processing {

    class VampireBodyDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<VampireBodyDetector>;

        explicit VampireBodyDetector(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  debug_contour_("Debug_contour", false, &parameters_),
                  look_for_rectangle_("Look_for_Rectangle", false, &parameters_),
                  min_area_("Min_area", 100, 1, 10000, &parameters_),
                  max_area_("Max_area", 1000, 1, 50000, &parameters_) {
            setName("VampireBodyDetector");
        }

        ~VampireBodyDetector() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                std::string objective;
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
                //cv::Mat originalImage = global_param_handler_.getOriginalImage();

                PerformanceEvaluator timer;
                timer.resetStartTime();

                contourList_t contours;

                //retrieveContours(image, contours);
                //std::cout << "Contours : " << contours.size() << std::endl;

                //retrieveOuterContours(image, contours);
                //std::cout << "Outer Contours : " << contours.size() << std::endl;

                retrieveAllContours(image, contours);
                ObjectFullData::FullObjectPtrVec objVec;
                //std::cout << "All Contours : " << contours.size() << std::endl << std::endl;
                for (int i = 0; i < contours.size(); i++) {
                    ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image,
                                                                                  reinterpret_cast<Contour &&>(contours[i]));
                    if (object.get() == nullptr) {
                        continue;
                    }

                    //AREA
                    //std::cout << "Area : " << object->getArea() << std::endl;

                    if (object->getArea() < min_area_() || object->getArea() > max_area_()) {
                        continue;
                    }

                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                    }

                    //std::cout << "Is rectangle : " << isRectangle(contours[i],20) << std::endl;

                    if (look_for_rectangle_() && isRectangle(contours[i], 20)) {

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objective = "vampire_torpedoes";

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
        }

    private:
        cv::Mat output_image_;

        Parameter<bool> enable_, debug_contour_, look_for_rectangle_;

        RangedParameter<double> min_area_, max_area_;
    };

}  // namespace proc_image_processing

#endif  //PROC_IMAGE_PROCESSING_VAMPIRE_BODY_DETECTOR_H
