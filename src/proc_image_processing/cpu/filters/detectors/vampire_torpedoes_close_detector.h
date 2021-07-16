/// \author csauvain
/// \date 20/07/19
/// TODO: Refactor code repetition

// FACTORY_GENERATOR_CLASS_NAME=VampireTorpedoesCloseDetector

#ifndef PROC_IMAGE_PROCESSING_VAMPIRE_TORPEDOES_CLOSE_DETECTOR_H
#define PROC_IMAGE_PROCESSING_VAMPIRE_TORPEDOES_CLOSE_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <math.h>
#include <memory>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

namespace proc_image_processing {

    class VampireTorpedoesCloseDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<VampireTorpedoesCloseDetector>;

        explicit VampireTorpedoesCloseDetector(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  debug_contour_("Debug_contour", false, &parameters_),
                  look_for_ellipse_("Look_for_Ellipse", false, &parameters_),
                  look_for_heart_("Look_for_Heart", false, &parameters_),
                  min_area_("Min_area", 5000, 1, 50000, &parameters_),
                  max_area_("Max_area", 100000, 1, 1000000, &parameters_) {
            setName("VampireTorpedoesCloseDetector");
        }

        ~VampireTorpedoesCloseDetector() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                std::string objective;
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
                //cv::Mat originalImage = global_params_.getOriginalImage();

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
                for (int i = 0, size = contours.size(); i < size; i++) {
                    ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image, contours[i]);
                    if (object.get() == nullptr) {
                        continue;
                    }

                    //AREA
                    // std::cout << object->getArea();

                    if (object->getArea() < min_area_() || object->getArea() > max_area_()) {
                        continue;
                    }

                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                    }

                    cv::Mat pointfs;
                    cv::Mat(contours[i]).convertTo(pointfs, CV_32F);
                    cv::RotatedRect box = cv::fitEllipse(pointfs);

                    float circleIndex;
                    float percentageFilled;

                    if (look_for_ellipse_()) {

                        circleIndex = getCircleIndex(contours[i]);

                        //std::cout << circleIndex << std::endl;

                        if (circleIndex < 0.7) {
                            continue;
                        }

                        pourcentageFilled = getPercentFilled(output_image_, box);

                        if (percentageFilled > 25) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objective = "vampire_torpedoes";
                    }

                    if (look_for_heart_()) {

                        circleIndex = getCircleIndex(contours[i]);

                        if (circleIndex > 0.9) {
                            continue;
                        }

                        pourcentageFilled = getPercentFilled(output_image_, box);

                        if (percentageFilled > 50) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objective = "heart_torpedoes";
                    }
                    objVec.push_back(object);
                }

                std::sort(objVec.begin(), objVec.end(),
                          [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                              return
                                      a->getArea() >
                                      b->getArea();
                          });

                if (!objVec.empty()) {
                    Target target;
                    ObjectFullData::Ptr object = objVec[0];
                    cv::Point center = object->getCenterPoint();
                    target.setTarget(objective, center.x, center.y, object->getWidth(), object->getHeight(),
                                     object->getRotRect().angle, image.rows, image.cols);
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

        Parameter<bool> enable_, debug_contour_, look_for_ellipse_, look_for_heart_;

        RangedParameter<double> min_area_, max_area_;

    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_VAMPIRE_TORPEDOES_CLOSE_DETECTOR_H
