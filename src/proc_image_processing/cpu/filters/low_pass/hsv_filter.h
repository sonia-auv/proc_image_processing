// FACTORY_GENERATOR_CLASS_NAME=BGRFilter

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class HSVFilter : public Filter
    {
    public:
        using Ptr = std::shared_ptr<HSVFilter>;

        explicit HSVFilter(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_return_original("Return Original Image", false, &parameters_, "Returned filtered Image or return original with mask"),
              m_hue("Hue", 128.0, 0.0, 255.0, &parameters_, "Hue ( Dominant Wavelength )"),
              m_saturation("Saturation", 128.0, 0.0, 255.0, &parameters_, "Saturation ( Purity / shades of the color )"),
              m_value("Value", 128.0, 0.0, 255.0, &parameters_, "Value ( Intensity )"),
              m_hue_thresh("Hue Threshold", 128.0, 0.0, 128.0, &parameters_, "Hue Range"),
              m_saturation_thresh("Saturation Threshold", 128.0, 0.0, 128.0, &parameters_, "Saturation Range"),
              m_value_thresh("Value Threshold", 128.0, 0.0, 128.0, &parameters_, "Value Range")
        {
            setName("HSVFilter");
        }

        ~HSVFilter() override = default;

        void apply(cv::Mat &image) override
        {
            cv::Mat hsvImage;
            cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
            cv::Vec3b hsvPixel(m_hue(), m_saturation(), m_value());

            cv::Scalar minHSV = cv::Scalar(hsvPixel.val[0] - m_hue_thresh(), hsvPixel.val[1] - m_saturation_thresh(), hsvPixel.val[2] - m_value_thresh());
            cv::Scalar maxHSV = cv::Scalar(hsvPixel.val[0] + m_hue_thresh(), hsvPixel.val[1] + m_saturation_thresh(), hsvPixel.val[2] + m_value_thresh());

            cv::Mat maskHSV, resultHSV;
            cv::inRange(hsvImage, minHSV, maxHSV, maskHSV);
            if (m_return_original())
            {
                cv::bitwise_and(image, image, resultHSV, maskHSV);
            }
            else
            {
                cv::bitwise_and(hsvImage, hsvImage, resultHSV, maskHSV);
            }
            resultHSV.copyTo(image);
        }

    private:
        Parameter<bool> m_return_original;
        RangedParameter<double> m_hue;
        RangedParameter<double> m_saturation;
        RangedParameter<double> m_value;
        RangedParameter<double> m_hue_thresh;
        RangedParameter<double> m_saturation_thresh;
        RangedParameter<double> m_value_thresh;
    };
}