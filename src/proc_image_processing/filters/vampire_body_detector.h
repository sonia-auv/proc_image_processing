//
// Created by csauvain on 20/07/19.
//

#ifndef PROC_IMAGE_PROCESSING_VAMPIRE_BODY_DETECTOR_H
#define PROC_IMAGE_PROCESSING_VAMPIRE_BODY_DETECTOR_H


#include <proc_image_processing/filters/filter.h>
#include <math.h>
#include <memory>
#include <proc_image_processing/algorithm/performance_evaluator.h>

namespace proc_image_processing {

    class VampireBodyDetector : public Filter {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        using Ptr = std::shared_ptr<VampireBodyDetector>;

        //============================================================================
        // P U B L I C   C / D T O R S

        explicit VampireBodyDetector(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                enable_("Enable", false, &parameters_),
                debug_contour_("Debug_contour", false, &parameters_),
                look_for_rectangle_("Look_for_Rectangle", false, &parameters_),
                min_area_("Min_area", 100, 1, 10000, &parameters_),
                max_area_("Max_area", 1000, 1, 50000, &parameters_) {
            SetName("VampireBodyDetector");}

        virtual ~VampireBodyDetector() {}

        //============================================================================
        // P U B L I C   M E T H O D S
        virtual void Execute(cv::Mat &image){
            if (enable_()) {
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }
            }

            if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
            cv::Mat originalImage = global_params_.getOriginalImage();

            PerformanceEvaluator timer;
            timer.UpdateStartTime();

            contourList_t contours;
            RetrieveOuterContours(image, contours);
            ObjectFullData::FullObjectPtrVec objVec;
            for (int i = 0, size = contours.size(); i < size; i++) {
                ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(originalImage, image, contours[i]);
                if (object.get() == nullptr) {
                    continue;
                }
                //AREA
                //ROS_DEBUG_STREAM("Area : " << object->GetArea());
                if (object->GetArea() < min_area_() || object->GetArea() > max_area_()) {
                    continue;
                }

                if (look_for_rectangle_() && !IsRectangle(contours[i], 10)) {
                    continue;
                }

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                }

                objVec.push_back(object);
            }

            std::sort(objVec.begin(), objVec.end(),
                      [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                          return a->GetArea() > b->GetArea();
                      });

            // Since we search only one buoy, get the biggest from sort function
            if (objVec.size() > 0) {
                Target target;
                ObjectFullData::Ptr object = objVec[0];
                cv::Point center = object->GetCenter();

                target.SetTarget("grab_vampire", center.x, center.y, object->GetLength(),
                                 object->GetLength(), object->GetRotatedRect().angle,
                                 image.rows, image.cols);
                NotifyTarget(target);
                if (debug_contour_()) {
                    cv::circle(output_image_, objVec[0]->GetCenter(), 3,
                               CV_RGB(0, 255, 0), 3);
                }
            }




            if (debug_contour_()) {
                output_image_.copyTo(image);
            }


    }




    private:
        //============================================================================
        // P R I V A T E   M E M B E R S
        cv::Mat output_image_;

        Parameter<bool> enable_, debug_contour_, look_for_rectangle_;

        RangedParameter<double> min_area_, max_area_;
    };

}  // namespace proc_image_processing

#endif  //PROC_IMAGE_PROCESSING_VAMPIRE_BODY_DETECTOR_H
