//
// Created by csauvain on 20/07/19.
// Il y a des repetitions de code a ameliorer
//

#ifndef PROC_IMAGE_PROCESSING_VAMPIRE_TORPIDOES_DETECTOR_H
#define PROC_IMAGE_PROCESSING_VAMPIRE_TORPIDOES_DETECTOR_H


#include <proc_image_processing/filters/filter.h>
#include <math.h>
#include <memory>
#include <proc_image_processing/algorithm/performance_evaluator.h>
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

namespace proc_image_processing {

    class VampireTorpidoesDetector : public Filter {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        using Ptr = std::shared_ptr<VampireTorpidoesDetector>;

        //============================================================================
        // P U B L I C   C / D T O R S

        explicit VampireTorpidoesDetector(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                enable_("Enable", false, &parameters_),
                debug_contour_("Debug_contour", false, &parameters_),
                look_for_ellipse_("Look_for_Ellipse", false, &parameters_),
                look_for_heart_("Look_for_Heart", false, &parameters_),
                min_area_("Min_area", 5000, 1, 100000, &parameters_),
                max_area_("Max_area", 100000, 1, 1000000, &parameters_) {
            SetName("VampireTorpidoesDetector");}

        virtual ~VampireTorpidoesDetector() {}

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

                RetrieveContours(image, contours);
                std::cout << "Contours : " << contours.size() << std::endl;

                //RetrieveOuterContours(image, contours);
                //std::cout << "Outer Contours : " << contours.size() << std::endl;

                RetrieveAllContours(image, contours);
                ObjectFullData::FullObjectPtrVec objVec;
                std::cout << "All Contours : " << contours.size() << std::endl << std::endl;
                for (int i = 0, size = contours.size(); i < size; i++) {
                    ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image, contours[i]);
                    if (object.get() == nullptr) {
                        continue;
                    }
                    //AREA

                   //std::cout << object->GetArea();

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

                    if (look_for_ellipse_()) {

                        circleIndex = CalculateCircleIndex(contours[i]);

                        std::cout << circleIndex << std::endl;

                        if (circleIndex < 0.7) {
                            continue;
                        }

                        pourcentageFilled = CalculatePourcentFilled(output_image_, box);

                        if (pourcentageFilled > 50) {
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        objectif = "vampire_torpedoes";

                    }

                    if (look_for_heart_()) {

                        circleIndex = CalculateCircleIndex(contours[i]);

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
                    target.SetTarget(objectif, center.x, center.y, object->GetLength(), object->GetLength(), object->GetRotatedRect().angle, image.rows, image.cols);
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

        Parameter<bool> enable_, debug_contour_, look_for_ellipse_, look_for_heart_;

        RangedParameter<double> min_area_, max_area_;

    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_VAMPIRE_TORPIDOES_DETECTOR_H
