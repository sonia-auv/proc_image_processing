//
// Created by olavoie on 11/29/17.
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
                min_area_("Min_area", 500, 1, 50000, &parameters_),
                max_area_("Max_area", 10000, 1, 50000, &parameters_) {
            SetName("VampireTorpidoesDetection");}

        virtual ~VampireTorpidoesDetector() {}

        //============================================================================
        // P U B L I C   M E T H O D S
        virtual void Execute(cv::Mat &image){
            if (enable_()) {
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
                //cv::Mat originalImage = global_params_.getOriginalImage();

                PerformanceEvaluator timer;
                timer.UpdateStartTime();

                contourList_t contours;
                RetrieveOuterContours(image, contours);
                ObjectFullData::FullObjectPtrVec objVec;
                for (int i = 0, size = contours.size(); i < size; i++) {
                    ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(output_image_, image, contours[i]);
                    if (object.get() == nullptr) {
                        continue;
                    }
                    //AREA
                    if (object->GetArea() < min_area_() || object->GetArea() > max_area_()) {
                        continue;
                    }

                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                    }

                    if (look_for_ellipse_()) {
                        //cv::Mat pointfs;
                        //cv::Mat(contours[i]).convertTo(pointfs,CV_32F);
                        //cv::RotatedRect box = cv::fitEllipse(pointfs);

                        //cv::drawContours(output_image_, contours, (int)i, cv::Scalar::all(255), 1, 8);

                        //cv::ellipse(output_image_, box, cv::Scalar(0,0,255), 1, CV_AA);
                        //cv::ellipse(output_image_, box.center, box.size*0.5f, box.angle, 0, 360, cv::Scalar(0,255,255), 1, CV_AA);

                        //cv::Point2f vtx[4];
                        //box.points(vtx);
                        //cv::Point2f center = box.center;
                        //cv::Size2f size_ellipse = box.size;

                        //Target current_ellipse_target;
                        //current_ellipse_target.SetTarget("vampire_torpidoes", center.x, center.y, size.);
                        //current_ellipse_target.SetTarget("vampire_torpidoes", object->GetCenter().x, object->GetCenter().y, size_ellipse.width, size_ellipse.height, object->GetAngle(), object->GetImageSize().height, object->GetImageSize().width, "test1", "test2");

                        //NotifyTarget(current_ellipse_target)




                        //for(int j = 0; j < 4; j++)
                        //    cv::line(output_image_, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
                        float circleIndex;
                        circleIndex = CalculateCircleIndex(contours[i]);

                        if (circleIndex > 0.9){
                            continue;
                        }

                        if (debug_contour_()) {
                            cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                        }

                        std::cout << circleIndex << std::endl;

                    }

                    //cv::SimpleBlobDetector();

                    objVec.push_back(object);
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
