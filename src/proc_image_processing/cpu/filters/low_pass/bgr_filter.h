// FACTORY_GENERATOR_CLASS_NAME=BGRFilter

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class BGRFilter : public Filter
    {
    public:
        using Ptr = std::shared_ptr<BGRFilter>;

        explicit BGRFilter(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_filterType("Filter Type", 0, 0, 3, &parameters_,"0=BGR, 1=LAB, 2=YCrCb, 3=HSV"),
              m_blue("Blue", 128.0, 0.0, 255.0, &parameters_, "How much Blue"),
              m_green("Green", 128.0, 0.0, 255.0, &parameters_, "How much Green"),
              m_red("Red", 128.0, 0.0, 255.0, &parameters_, "How much Red"),
              m_blue_thresh("Blue Threshold", 128.0, 0.0, 128.0, &parameters_, "Blue Range"),
              m_green_thresh("Green Threshold", 128.0, 0.0, 128.0, &parameters_, "Green Range"),
              m_red_thresh("Red Threshold", 128.0, 0.0, 128.0, &parameters_, "Red Range")
        {
            setName("BGRFilter");
        }

        ~BGRFilter() override = default;

        void apply(cv::Mat &image) override
        {
            cv::Vec3b pixels(m_blue(), m_green(), m_red());
            cv::Mat resultImage;

            switch (m_filterType())
            {
            case 1: // BGR 2 LAB Filter
                applyLABFilter(image, pixels);
                break;
            case 2: // BGR 2 YCrCb Filter
                applyYCrCbFilter(image, pixels);
                break;
            case 3: // BGR 2 HSV Filter
                applyHSVFilter(image, pixels);
                break;
            default: // Basic BGR Filter
                applyBGRFilter(image, pixels);
                break;
            }
        }

    private:
        void applyBGRFilter(cv::Mat &image, cv::Vec3b &pixels)
        {
            cv::Scalar minBGR = getMinThreshold(pixels);
            cv::Scalar maxBGR = getMaxThreshold(pixels);

            applyMask(image, minBGR, maxBGR).copyTo(image);
        }

        void applyLABFilter(cv::Mat &image, cv::Vec3b &pixels)
        {
            cv::Mat labImage;
            cv::cvtColor(image, labImage, cv::COLOR_BGR2Lab);

            cv::Mat3b bgr(pixels);

            cv::Mat3b lab;
            cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
            cv::Vec3b labPixel(lab.at<cv::Vec3b>(0, 0));

            cv::Scalar minLAB = getMinThreshold(labPixel);
            cv::Scalar maxLAB = getMaxThreshold(labPixel);

            applyMask(labImage, minLAB, maxLAB).copyTo(image);
        }

        void applyYCrCbFilter(cv::Mat &image, cv::Vec3b &pixels)
        {
            cv::Mat ycbImage;
            cv::cvtColor(image, ycbImage, cv::COLOR_BGR2YCrCb);

            cv::Mat3b bgr(pixels);

            cv::Mat3b ycb;
            cv::cvtColor(bgr, ycb, cv::COLOR_BGR2YCrCb);
            cv::Vec3b ycbPixel(ycb.at<cv::Vec3b>(0, 0));

            cv::Scalar minYCB = getMinThreshold(ycbPixel);
            cv::Scalar maxYCB = getMaxThreshold(ycbPixel);

            applyMask(ycbImage, minYCB, maxYCB).copyTo(image);
        }

        void applyHSVFilter(cv::Mat &image, cv::Vec3b &pixels)
        {
            cv::Mat hsvImage;
            cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

            cv::Mat3b bgr(pixels);

            cv::Mat3b hsv;
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
            cv::Vec3b hsvPixel(hsv.at<cv::Vec3b>(0, 0));

            cv::Scalar minHSV = getMinThreshold(hsvPixel);
            cv::Scalar maxHSV = getMaxThreshold(hsvPixel);

            applyMask(hsvImage, minHSV, maxHSV).copyTo(image);
        }

        cv::Scalar getMinThreshold(cv::Vec3b &pixels)
        {
            return cv::Scalar(pixels.val[0] - m_blue_thresh(), pixels.val[1] - m_green_thresh(), pixels.val[2] - m_red_thresh());
        }

        cv::Scalar getMaxThreshold(cv::Vec3b &pixels)
        {
            return cv::Scalar(pixels.val[0] + m_blue_thresh(), pixels.val[1] + m_green_thresh(), pixels.val[2] + m_red_thresh());
        }

        cv::Mat applyMask(cv::Mat &image, cv::Scalar min, cv::Scalar max)
        {
            cv::Mat mask, result;
            cv::inRange(image, min, max, mask);
            cv::bitwise_and(image, image, result, mask);
            return result;
        }

        RangedParameter<int> m_filterType;
        RangedParameter<double> m_blue;
        RangedParameter<double> m_green;
        RangedParameter<double> m_red;
        RangedParameter<double> m_blue_thresh;
        RangedParameter<double> m_green_thresh;
        RangedParameter<double> m_red_thresh;
    };
}