//
// Created by csauvain on 20/07/19.
// Il y a des repetitions de code a ameliorer
//

#ifndef PROC_IMAGE_PROCESSING_VAMPIRE_TORPIDOES_DETECTOR_CLOSE_H
#define PROC_IMAGE_PROCESSING_VAMPIRE_TORPIDOES_DETECTOR_CLOSE_H


#include <proc_image_processing/filters/filter.h>
#include <math.h>
#include <memory>
#include <proc_image_processing/algorithm/performance_evaluator.h>
#include <proc_image_processing/algorithm/general_function.h>
#include <iostream>

namespace proc_image_processing {

    class VampireTorpidoesDetectorClose : public Filter {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        using Ptr = std::shared_ptr<VampireTorpidoesDetectorClose>;

        //============================================================================
        // P U B L I C   C / D T O R S

        explicit VampireTorpidoesDetectorClose(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                enable_("Enable", false, &parameters_),
                debug_contour_("Debug_contour", false, &parameters_),
                distance_detection_("Distance_detection", false, &parameters_),
                look_for_ellipse_("Look_for_Ellipse", false, &parameters_),
                look_for_heart_("Look_for_Heart", false, &parameters_),
                text_on_target_("Text_on_target", false, & parameters_),
                min_area_("Min_area", 5000, 1, 50000, &parameters_),
                max_area_("Max_area", 100000, 1, 1000000, &parameters_),
                text_size_("Text_size", 1, 0, 15, & parameters_),
                real_object_height_("Real_object_height (mm)", 50, 0, 2000, & parameters_),
                text_thickness_("Text_thickness", 1, 0, 15, & parameters_),
                text_pos_x_("Text_pos_x", 500, 0, 2000, & parameters_),
                text_pos_y_("Text_pos_y", 500, 0, 2000, & parameters_) {
            SetName("VampireTorpidoesDetectorClose");}

        virtual ~VampireTorpidoesDetectorClose() {}

        //============================================================================
        // P U B L I C   M E T H O D S
        virtual void Execute(cv::Mat &image){
            if (enable_()) {
                std::string objectif;
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
                //cv::Mat originalImage = global_params_.getOriginalImage();

                PerformanceEvaluator timer;
                timer.UpdateStartTime();

                contourList_t contours;

                RetrieveAllContours(image, contours);
                ObjectFullData::FullObjectPtrVec objVec;
                
                for (int i = 0, size = contours.size(); i < size; i++) {
                    ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image, contours[i]);
                    if (object.get() == nullptr) {
                        continue;
                    }
                    
                    if (object->GetArea() < min_area_() || object->GetArea() > max_area_()) {
                        continue;
                    }

                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                    }

                    cv::Mat pointfs;
                    cv::Mat(contours[i]).convertTo(pointfs,CV_32F);
                    cv::RotatedRect box = cv::fitEllipse(pointfs);

                    float circleIndex;
                    float pourcentageFilled;

                    circleIndex = CalculateCircleIndex(contours[i]);

                    if (look_for_ellipse_()) {
                        if (circleIndex < 0.7) {
                            continue;
                        }

                        pourcentageFilled = CalculatePourcentFilled(output_image_, box);

                        if (pourcentageFilled > 25) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objectif = "vampire_torpedoes";

                    }

                    if (look_for_heart_()) {
                        if (circleIndex > 0.9) {
                            continue;
                        }

                        pourcentageFilled = CalculatePourcentFilled(output_image_, box);

                        if (pourcentageFilled > 50) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objectif = "heart_torpedoes";

                    }

                    objVec.push_back(object);
                }

                std::sort(objVec.begin(), objVec.end(), [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool { return a->GetArea() > b->GetArea();});

                if (objVec.size() > 0) {
                    Target target;
                    ObjectFullData::Ptr object = objVec[0];
                    cv::Point center = object->GetCenter();
                    float distance = 0.0;

                    if (distance_detection_()){
                        //std::string distance;
                        if (text_on_target_()) {
                            center = object->GetCenter();
                        }
                        else
                        {
                            center.x = text_pos_x_();
                            center.y = text_pos_y_();
                        }
                        
                        distance = TargetDistance(real_object_height_(), object->GetHeight(), image.rows);
                        std::string str_distance = objectif + " : " + std::to_string(distance) + " m";

                        cv::putText(output_image_,str_distance, center,cv::FONT_HERSHEY_SIMPLEX, text_size_(),CV_RGB(0,255,0),text_thickness_(),cv::FILLED,false);
                    }

                    target.SetTarget(objectif, center.x, center.y, object->GetWidth(), object->GetHeight(), object->GetRotatedRect().angle, distance, image.rows, image.cols);
                    NotifyTarget(target);
                    if (debug_contour_()) {
                        cv::circle(output_image_, objVec[0]->GetCenter(), 3, CV_RGB(0,255,0),3);
                    }
                    
                }

                if (debug_contour_()) {
                    output_image_.copyTo(image);
                }

            }

    }

    private:
        //============================================================================
        // P R I V A T E   M E M B E R S
        cv::Mat output_image_;

        Parameter<bool> enable_, debug_contour_, distance_detection_, look_for_ellipse_, look_for_heart_, text_on_target_;

        RangedParameter<double> min_area_, max_area_, text_size_, real_object_height_;

        RangedParameter<int> text_thickness_, text_pos_x_, text_pos_y_;

    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_VAMPIRE_TORPIDOES_DETECTOR_CLOSE_H
