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

        explicit GateSymbolDetector(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_tol_white("Tol. white", 0, 0, 255, &parameters_,"Tolerance in the detection of white"),
              m_kernel("Size of kernels", 0, 0, 20, &parameters_, "Size of kernels"),
              m_thres("Threshold", 0, 0, 255, &parameters_, "Threshold between white and black"),
              m_debug("Debug state", 0, 0, 2, &parameters_, "0=NoDebug, 1=BGRFilter, 2=result")
        {
            setName("GateSymbolDetector");
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

            params.filterByArea=false;
            params.filterByConvexity=false;
            params.filterByCircularity=false;

            params.filterByInertia=false;
            params.maxInertiaRatio=0.6;

            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

            std::vector<cv::KeyPoint> keypoints_earth;
            detector->detect(result, keypoints_earth);

            cv::drawKeypoints(image, keypoints_earth, image, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            params.maxInertiaRatio=1;
            params.minInertiaRatio=0.6;
            detector = cv::SimpleBlobDetector::create(params);

            std::vector<cv::KeyPoint> keypoints_aby;
            detector->detect(result, keypoints_aby);

            //cv::drawKeypoints(image, keypoints_aby, image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
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
    };
}