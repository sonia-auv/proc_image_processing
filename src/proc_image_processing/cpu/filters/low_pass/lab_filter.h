// FACTORY_GENERATOR_CLASS_NAME=BGRFilter

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class LABFilter : public Filter
    {
    public:
        using Ptr = std::shared_ptr<LABFilter>;

        explicit LABFilter(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_return_original("Return Original Image", false, &parameters_, "Returned filtered Image or return original with mask"),
              m_lightness("Lightness", 128.0, 0.0, 255.0, &parameters_, "Lightness (Intensity)"),
              m_alpha("Green - Magenta", 128.0, 0.0, 255.0, &parameters_, "olor component ranging from Green to Magenta"),
              m_beta("Blue - Yellow", 128.0, 0.0, 255.0, &parameters_, "color component ranging from Blue to Yellow"),
              m_lightness_thresh("Lightness Threshold", 128.0, 0.0, 128.0, &parameters_, "Lightness Range"),
              m_alpha_thresh("Green - Magenta Threshold", 128.0, 0.0, 128.0, &parameters_, "Green - Magenta Range"),
              m_beta_thresh("Blue - Yellow Threshold", 128.0, 0.0, 128.0, &parameters_, "Blue - Yellow Range")
        {
            setName("LABFilter");
        }

        ~LABFilter() override = default;

        void apply(cv::Mat &image) override
        {
            cv::Mat labImage;
            cv::cvtColor(image, labImage, cv::COLOR_BGR2Lab);
            cv::Vec3b labPixel(m_lightness(), m_alpha(), m_beta());

            cv::Scalar minLAB = cv::Scalar(labPixel.val[0] - m_lightness_thresh(), labPixel.val[1] - m_alpha_thresh(), labPixel.val[2] - m_beta_thresh());
            cv::Scalar maxLAB = cv::Scalar(labPixel.val[0] + m_lightness_thresh(), labPixel.val[1] + m_alpha_thresh(), labPixel.val[2] + m_beta_thresh());

            cv::Mat maskLAB, resultLAB;
            cv::inRange(labImage, minLAB, maxLAB, maskLAB);
            if (m_return_original())
            {
                cv::bitwise_and(image, image, resultLAB, maskLAB);
            }
            else
            {
                cv::bitwise_and(labImage, labImage, resultLAB, maskLAB);
            }
            resultLAB.copyTo(image);
        }

    private:
        Parameter<bool> m_return_original;
        RangedParameter<double> m_lightness;
        RangedParameter<double> m_alpha;
        RangedParameter<double> m_beta;
        RangedParameter<double> m_lightness_thresh;
        RangedParameter<double> m_alpha_thresh;
        RangedParameter<double> m_beta_thresh;
    };
}