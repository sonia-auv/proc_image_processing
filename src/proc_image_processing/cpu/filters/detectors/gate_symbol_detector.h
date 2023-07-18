// FACTORY_GENERATOR_CLASS_NAME=GateSymbolDetector

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class GateSymbolDetector : public Filter
    {
    public:
        using Ptr = std::shared_ptr<GateSymbolDetector>;

        explicit GateSymbolDetector(const GlobalParameterHandler &globalParams, ros::NodeHandlePtr nhp)
            : Filter(globalParams, nhp),
              m_tol_white("Tol. white", 0, 0, 255, &parameters_,"Tolerance in the detection of white"),
              m_kernel("Size of kernels", 1, 1, 20, &parameters_, "Size of kernels"),
              m_thres("Threshold", 128, 0, 255, &parameters_, "Threshold between white and black"),
              m_debug("Debug state", 0, 0, 2, &parameters_, "0=NoDebug, 1=BGRFilter, 2=result")
        {
            setName("GateSymbolDetector");
            m_earth_pub = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/earth_target", 100);
            m_aby_pub = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/aby_target", 100);
        }

        ~GateSymbolDetector() override = default;

        /**
         * Apply the filter
         * 
         * @param image The image to apply the changes to.
        */
        void apply(cv::Mat &image) override
        {
            cv::Scalar min = cv::Scalar(255-m_tol_white(), 255-m_tol_white(), 255-m_tol_white());
            cv::Scalar max = cv::Scalar(255, 255, 255);

            cv::Mat back;
            image.copyTo(back);
            back.setTo(255);

            cv::Mat mask;
            cv::inRange(image, min, max, mask);
        
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(m_kernel(), m_kernel()), cv::Point(-1, -1));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 5);
            cv::erode(mask, mask, kernel);

            cv::Mat result;
            cv::bitwise_and(image, image, result, mask);

            //cv::bitwise_not(mask, mask);
            cv::bitwise_and(back, 0, back, mask);

            cv::bitwise_or(result, back, result);
            cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
            cv::threshold(result, result, m_thres(), 255, cv::THRESH_BINARY);



            cv::SimpleBlobDetector::Params params;

            params.filterByArea=true;
            params.minArea=100;
            params.filterByConvexity=false;
            params.filterByCircularity=false;

            params.filterByInertia=true;
            params.maxInertiaRatio=0.6;

            cv::Ptr<cv::SimpleBlobDetector> detectorE = cv::SimpleBlobDetector::create(params);

            std::vector<cv::KeyPoint> keypoints_earth;
            detectorE->detect(result, keypoints_earth);

            cv::drawKeypoints(image, keypoints_earth, image, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            std::sort(
                    keypoints_earth.begin(),
                    keypoints_earth.end(),
                    [](cv::KeyPoint a, cv::KeyPoint b) -> bool {
                        return a.size > b.size;
                    }
            );

            geometry_msgs::Pose2D objet;
            sonia_common::FilterchainTarget target;

            if(keypoints_earth.size()>=1){
                objet.x = keypoints_earth[0].pt.x;
                objet.y = keypoints_earth[0].pt.y;
                target.obj_ctr = objet;
                target.obj_size = keypoints_earth[0].size;
                m_earth_pub.publish(target);
            }
            

            params.maxInertiaRatio=1;
            params.minInertiaRatio=0.6;
            cv::Ptr<cv::SimpleBlobDetector> detectorA = cv::SimpleBlobDetector::create(params);

            std::vector<cv::KeyPoint> keypoints_aby;
            detectorA->detect(result, keypoints_aby);

            cv::drawKeypoints(image, keypoints_aby, image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            std::sort(
                    keypoints_aby.begin(),
                    keypoints_aby.end(),
                    [](cv::KeyPoint a, cv::KeyPoint b) -> bool {
                        return a.size > b.size;
                    }
            );

            if(keypoints_aby.size()>=1){
               objet.x = keypoints_aby[0].pt.x;
                objet.y = keypoints_aby[0].pt.y;
                target.obj_ctr = objet;
                target.obj_size = keypoints_aby[0].size;
                m_aby_pub.publish(target); 
            }
            

            switch (m_debug()) {
            case 1 :
                image = mask;
                break;
            case 2 :
                image = result;
                break;
            default : 
                break;
            }
        }

    private:
        RangedParameter<int> m_tol_white;
        RangedParameter<int> m_debug;
        RangedParameter<int> m_kernel;
        RangedParameter<int> m_thres;

        ros::Publisher m_earth_pub;
        ros::Publisher m_aby_pub;

    };
}