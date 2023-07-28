// FACTORY_GENERATOR_CLASS_NAME=PipeDetector

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>
#include <sonia_common/FilterchainTargetAngle.h>

namespace proc_image_processing
{
    class PipeDetector : public Filter
    {
    public:
        using Ptr = std::shared_ptr<PipeDetector>;

        explicit PipeDetector(const GlobalParameterHandler &globalParams, ros::NodeHandlePtr nhp)
            : Filter(globalParams, nhp),
              m_show_angle("Show angle", false, &parameters_,"Choose to write the angle on the image or not")
        {
            setName("PipeDetector");
            m_pipe_pub = this->nhp_->advertise<sonia_common::FilterchainTargetAngle>("/proc_image_processing/pipe_target", 100);
        }

        ~PipeDetector() override = default;

        /**
         * Apply the filter
         * 
         * @param image The image to apply the changes to.
        */
        void apply(cv::Mat &image) override
        {
            std::vector<std::vector<cv::Point> > contours;
            cv::Mat contourOutput = image.clone();
            cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

            if (image.channels() == 1) {
                cv::cvtColor(image, image, CV_GRAY2BGR);
            }

            cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);

            
            std::sort(
                    contours.begin(),
                    contours.end(),
                    [](std::vector<cv::Point> a, std::vector<cv::Point> b) -> bool {
                        return a.size() > b.size();
                    }
            );

            if(contours.size() > 1){
                cv:: RotatedRect rect = cv::minAreaRect(contours[1]);
                double angle = rect.angle;
                cv::Rect2f rect2 = rect.boundingRect2f();
                    
                if(rect2.width > rect2.height){
                    angle = -(angle + 90);
                }

                if(angle > 45){
                    angle = 90 - angle;
                }

                if(angle < -45){
                    angle = angle + 90;
                }

                if(m_show_angle()){
                    cv::putText(image, std::to_string((int)angle), rect.center, CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
                }

                geometry_msgs::Pose2D objet;
                sonia_common::FilterchainTargetAngle target;

                objet.x = rect.center.x;
                objet.y = rect.center.y;
                target.obj_ctr = objet;
                target.obj_size = rect2.area();
                target.obj_angle = angle;
                m_pipe_pub.publish(target);
            }
            

            
        }

    private:
       
        Parameter<bool> m_show_angle;

        ros::Publisher m_pipe_pub;
    };
}