// FACTORY_GENERATOR_CLASS_NAME=GateAngleDetector

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>
#include <sonia_common/FilterchainTargetAngle.h>

namespace proc_image_processing
{
    class GateAngleDetector : public Filter
    {
    public:
        using Ptr = std::shared_ptr<GateAngleDetector>;

        explicit GateAngleDetector(const GlobalParameterHandler &globalParams, ros::NodeHandlePtr nhp)
            : Filter(globalParams, nhp),
              m_show_info("Show rectangle", false, &parameters_,"Choose to display the rectangle or not")
        {
            setName("GateAngleDetector");
            m_pub = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/gate_angle", 100);
        }

        ~GateAngleDetector() override = default;

        /**
         * Apply the filter
         * 
         * @param image The image to apply the changes to.
        */
        void apply(cv::Mat &image) override
        {
            std::vector<std::vector<cv::Point>> contours;
            cv::Mat contourOutput = image.clone();
            cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

            if (image.channels() == 1) {
                cv::cvtColor(image, image, CV_GRAY2BGR);
            }

            std::sort(
                    contours.begin(),
                    contours.end(),
                    [](std::vector<cv::Point> a, std::vector<cv::Point> b) -> bool {
                        return a.size() > b.size();
                    }
            );
            
            if(contours.size() > 1){
                cv::RotatedRect rect = cv::minAreaRect(contours[1]);
                double angle = rect.angle;
                cv::Rect2f rect2 = rect.boundingRect2f();

                cv::Point2f points[4];
                rect.points(points);

                try{
                    if(m_show_info()){
                        for(int j=0; j<3; j++){
                            cv::line(image, points[j], points[j+1], cv::Scalar(0, 0, 255));
                        }
                        cv::line(image, points[0], points[3], cv::Scalar(0, 0, 255));
                    }
                }catch(std::exception e){}
                
                geometry_msgs::Pose2D objet;
                sonia_common::FilterchainTarget target;

                objet.x = rect2.width;
                objet.y = rect2.height;
                target.obj_ctr = objet;
                target.obj_size = rect2.width/rect2.height;
                m_pub.publish(target);
            }

            
        }

    private:
       
        Parameter<bool> m_show_info;

        ros::Publisher m_pub;
    };
}