// FACTORY_GENERATOR_CLASS_NAME=ObstacleDetector

#ifndef PROC_IMAGE_PROCESSING_SHAPE_DETECTOR_H
#define PROC_IMAGE_PROCESSING_SHAPE_DETECTOR_H

#include <proc_image_processing/cpu/filters/filter.h>
#include <cmath>
#include <memory>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>

//The purpose of this code is to be better than "Obstacle detector" on trapezes shapes
// But finally obstacle detector was good enough so this one was left abandonned. 



namespace proc_image_processing {

    class ShapeDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<ShapeDetector>;

        explicit ShapeDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  look_for_filled_("Look for filled", false, &parameters_),
                  obstacle_("Obstacle", "", &parameters_),
                  min_area_("Minimum area", 100, 1, 10000, &parameters_, "Min area"),
                  max_area_("Maximum area", 1000, 1, 100000, &parameters_, "Max area"),
                  min_div_("Min div", 0.5, 0, 1, &parameters_, "Min div"),
                  max_div_("Max div", 0.5, 0, 1, &parameters_, "Max div") {
            setName("ShapeDetector");
        }

        ~ShapeDetector() override = default;

        void apply(cv::Mat &image) override {
            std::string desc1 = "";
            std::string desc2 = "";
            double percentFilled;
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
                }
                

                float area_on_length = sqrt(cv::contourArea(contours[i]))/cv::arcLength(contours[i], true );


                if (0.2 < area_on_length && area_on_length < 0.22) {
                    desc1 = "Triangle";
                }

                if (0.245 < area_on_length && area_on_length < 0.248) {
                    desc1 = "Rectangle";
                }

                if (0.14 < area_on_length && area_on_length < 0.15) {
                    desc1 = "Star";
                }

                if (0.105 < area_on_length && area_on_length < 0.135) {
                    desc1 = "Trapeze";
                }

                if (min_div_() < area_on_length && area_on_length < max_div_()) {
                    desc1 = "Other";
                    desc2 = std::to_string(area_on_length);
                }
                
                if (debug_contour_() && desc1 != ""){
                    cv::drawContours(output_image_, contours, i, CV_RGB(0, 0, 255), 2); 
                }
             

                if (contours[i].size() >=5) {
                    cv::Mat pointfs;
                    cv::Mat(contours[i]).convertTo(pointfs, CV_32F);
                    cv::RotatedRect box = cv::fitEllipse(pointfs);

                    double circleIndex = getCircleIndex(contours[i]);
                    double percentFilled = getPercentFilled(output_image_, box);
                    
                    if (circleIndex > 50) {
                        desc1 = "Circle";
                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        if (percentFilled > 50) {
                           desc1 = "Filled" + desc1;
                        }
                        else {
                           desc1 = "Empty" + desc1;
                        }
                        percentFilled = getPercentFilled(output_image_, box);
                        desc2 = std::to_string(percentFilled);
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
                        obstacle_(),
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
        Parameter<bool> look_for_filled_;
        Parameter<std::string> obstacle_;
        RangedParameter<double> min_area_;
        RangedParameter<double> max_area_;
        RangedParameter<double> max_div_;
        RangedParameter<double> min_div_;
    };

}  // namespace proc_image_processing

#endif  //PROC_IMAGE_PROCESSING_SHAPE_DETECTOR_H
