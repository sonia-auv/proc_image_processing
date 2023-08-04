// FACTORY_GENERATOR_CLASS_NAME=CornersDetector

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>
#include <sonia_common/FilterchainTarget.h>


namespace proc_image_processing
{
    class CornersDetector : public Filter
    {
    public:
        using Ptr = std::shared_ptr<CornersDetector>;

        explicit CornersDetector(const GlobalParameterHandler &globalParams, ros::NodeHandlePtr nhp)
            : Filter(globalParams, nhp)
        {
            setName("CornersDetector");
            m_pub_tl = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/tl", 100);
            m_pub_tr = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/tr", 100);
            m_pub_bl = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/bl", 100);
            m_pub_br = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/br", 100);
        }

        ~CornersDetector() override = default;

        /**
         * Apply the filter
         * 
         * @param image The image to apply the changes to.
        */
        void apply(cv::Mat &image) override
        {
            std::vector<std::vector<cv::Point>> contours;
           cv::findContours(image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

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
                
                geometry_msgs::Pose2D objet;
                sonia_common::FilterchainTarget target;

                cv::Point2f tl = points[0]+(points[2]-points[0])/4;
                cv::Point2f tr = points[0]+(points[2]-points[0])/4*3;
                cv::Point2f bl = points[1]+(points[3]-points[1])/4;
                cv::Point2f br = points[1]+(points[3]-points[1])/4*3;

                cv::circle(image, tl, 6, cv::Scalar(0, 255, 0), CV_FILLED);
                cv::circle(image, tr, 6, cv::Scalar(0, 255, 0), CV_FILLED);
                cv::circle(image, bl, 6, cv::Scalar(0, 255, 0), CV_FILLED);
                cv::circle(image, br, 6, cv::Scalar(0, 255, 0), CV_FILLED);

                objet.x = tl.x;
                objet.y = tl.y;
                target.obj_ctr = objet;
                target.obj_size = rect.size.width*rect.size.height;
                m_pub_tl.publish(target);

                objet.x = tr.x;
                objet.y = tr.y;
                target.obj_ctr = objet;
                target.obj_size = rect.size.width*rect.size.height;
                m_pub_tr.publish(target);

                objet.x = bl.x;
                objet.y = bl.y;
                target.obj_ctr = objet;
                target.obj_size = rect.size.width*rect.size.height;
                m_pub_bl.publish(target);

                objet.x = br.x;
                objet.y = br.y;
                target.obj_ctr = objet;
                target.obj_size = rect.size.width*rect.size.height;
                m_pub_br.publish(target);
            }            
        }

    private:

        ros::Publisher m_pub_tl;
        ros::Publisher m_pub_tr;
        ros::Publisher m_pub_bl;
        ros::Publisher m_pub_br;
    };
}