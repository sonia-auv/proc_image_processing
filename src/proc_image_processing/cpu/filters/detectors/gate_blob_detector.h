// FACTORY_GENERATOR_CLASS_NAME=GateBlobDetector

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>
#include <ros/ros.h>
#include <sonia_common/FilterchainTarget.h>

namespace proc_image_processing
{
    class GateBlobDetector : public Filter
    {
    public:
        using Ptr = std::shared_ptr<GateBlobDetector>;

        explicit GateBlobDetector(const GlobalParameterHandler &globalParams, ros::NodeHandle &nh_)
            : Filter(globalParams, nh_),
              m_tol_red("Tol. red", 0, 0, 255, &parameters_,"Tolerance of red in red detection"),
              m_tol_gb("Tol. for other colors", 0, 0, 255, &parameters_, "Tolerance of green and blue in red detection"),
              m_tol_black("Tol. black", 0, 0, 255, &parameters_,"Tolerance in the detection of black"),
              m_debug("Debug state", 0, 0, 2, &parameters_, "0=NoDebug, 1=RedFilter, 2=BlackFilter")
        {
            setName("GateBlobDetector");
            m_target_pub = this->nh_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/gate_target", 100);
        }

        ~GateBlobDetector() override = default;

        /**
         * Apply the filter
         * 
         * @param image The image to apply the changes to.
        */
        void apply(cv::Mat &image) override
        {
            cv::Scalar minr = cv::Scalar(0, 0, 255-m_tol_red());
            cv::Scalar maxr = cv::Scalar(m_tol_gb(), m_tol_gb(), 255);

            cv::Mat maskr;
            cv::inRange(image, minr, maxr, maskr);
            cv::bitwise_not(maskr, maskr);

            cv::Scalar minb = cv::Scalar(0, 0, 0);
            cv::Scalar maxb = cv::Scalar(m_tol_black(), m_tol_black(), m_tol_black());

            cv::Mat maskb;
            cv::inRange(image, minb, maxb, maskb);
            cv::bitwise_not(maskb, maskb);

            cv::SimpleBlobDetector::Params params;

            params.filterByArea=true;
            params.minArea=500;

            params.filterByConvexity=true;
            params.minConvexity=0.87;

            params.filterByCircularity=false;
            params.filterByInertia=false;

            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

            std::vector<cv::KeyPoint> keypointsr;
            detector->detect(maskr, keypointsr);

            std::vector<cv::KeyPoint> keypointsb;
            detector->detect(maskb, keypointsb);

            double dif;
            std::vector<std::vector<cv::KeyPoint>> gate(5, std::vector<cv::KeyPoint>(2)); 

            for(int i=0; i<keypointsb.size(); i++) {
                gate[i][0] = keypointsb[i];
                dif = 1300.0;
                for(int j=0; j<keypointsr.size(); j++) {
                    if(dif> abs(keypointsb[i].pt.x - keypointsr[j].pt.x)) {
                        dif = abs(keypointsb[i].pt.x - keypointsr[j].pt.x);
                        gate[i][1] = keypointsr[j];
                    }
                }
            }

            double angle = 0.785;
            cv::Point vecteur;

            for(int i=0; i<2; i++) {
                if(gate[i][1].size > 0) {
                    if(gate[i][0].pt.y < gate[i][1].pt.y) {
                        vecteur = cv::Point(gate[i][0].pt.x - gate[i][1].pt.x, gate[i][0].pt.y - gate[i][1].pt.y);
                        vecteur = cv::Point(cos(angle)*vecteur.x-sin(angle)*vecteur.y, sin(angle)*vecteur.x+cos(angle)*vecteur.y);
                        vecteur = cv::Point((int)(vecteur.x*1.6 + gate[i][1].pt.x), (int)(vecteur.y*1.6 + gate[i][1].pt.y));
                    }else{
                        vecteur = cv::Point(gate[i][1].pt.x - gate[i][0].pt.x, gate[i][1].pt.y - gate[i][0].pt.y);
                        vecteur = cv::Point(cos(-angle)*vecteur.x-sin(-angle)*vecteur.y, sin(-angle)*vecteur.x+cos(-angle)*vecteur.y);
                        vecteur = cv::Point((int)(vecteur.x*1.6 + gate[i][0].pt.x), (int)(vecteur.y*1.6 + gate[i][0].pt.y));
                    }

                    cv::circle(image, vecteur, 8, cv::Scalar(0, 255, 0), CV_FILLED);
                }
            }

            cv::drawKeypoints(image, keypointsb, image, cv::Scalar(0, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(image, keypointsr, image, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            switch (m_debug()) {
            case 1:
                image = maskr;
                break;
            case 2:
                image = maskb;
                break;
            default:
                break;
            }



               
        }

    private:
        RangedParameter<int> m_tol_red;
        RangedParameter<int> m_tol_gb;
        RangedParameter<int> m_tol_black;
        RangedParameter<int> m_debug;

        ros::Publisher m_target_pub;

    };
}