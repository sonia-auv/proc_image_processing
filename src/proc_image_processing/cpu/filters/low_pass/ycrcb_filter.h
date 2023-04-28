// FACTORY_GENERATOR_CLASS_NAME=BGRFilter

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class YCrCbFilter : public Filter
    {
    public:
        using Ptr = std::shared_ptr<YCrCbFilter>;

        explicit YCrCbFilter(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_return_original("Return Original Image", false, &parameters_, "Returned filtered Image or return original with mask"),
              m_luma("Y", 128.0, 0.0, 255.0, &parameters_, "Luminance or Luma component obtained from RGB after gamma correction"),
              m_chromiance_red("Cr", 128.0, 0.0, 255.0, &parameters_, "R - Y ( how far is the red component from Luma )"),
              m_chromiance_blue("Cb", 128.0, 0.0, 255.0, &parameters_, "B - Y ( how far is the blue component from Luma )"),
              m_luma_thresh("Y Threshold", 128.0, 0.0, 128.0, &parameters_, "Luminance Range"),
              m_cr_thresh("Cr Threshold", 128.0, 0.0, 128.0, &parameters_, "Cr Range"),
              m_cb_thresh("Cb Threshold", 128.0, 0.0, 128.0, &parameters_, "Cb Range")
        {
            setName("YCrCbFilter");
        }

        ~YCrCbFilter() override = default;

        void apply(cv::Mat &image) override
        {
            cv::Mat ycbImage;
            cv::cvtColor(image, ycbImage, cv::COLOR_BGR2YCrCb);
            cv::Vec3b ycbPixel(m_luma(), m_chromiance_red(), m_chromiance_blue());

            cv::Scalar minYCB = cv::Scalar(ycbPixel.val[0] - m_luma_thresh(), ycbPixel.val[1] - m_cr_thresh(), ycbPixel.val[2] - m_cb_thresh());
            cv::Scalar maxYCB = cv::Scalar(ycbPixel.val[0] + m_luma_thresh(), ycbPixel.val[1] + m_cr_thresh(), ycbPixel.val[2] + m_cb_thresh());

            cv::Mat maskYCB, resultYCB;
            cv::inRange(ycbImage, minYCB, maxYCB, maskYCB);
            if (m_return_original())
            {
                cv::bitwise_and(image, image, resultYCB, maskYCB);
            }
            else
            {
                cv::bitwise_and(ycbImage, ycbImage, resultYCB, maskYCB);
            }
            resultYCB.copyTo(image);
        }

    private:
        Parameter<bool> m_return_original;
        RangedParameter<double> m_luma;
        RangedParameter<double> m_chromiance_red;
        RangedParameter<double> m_chromiance_blue;
        RangedParameter<double> m_luma_thresh;
        RangedParameter<double> m_cr_thresh;
        RangedParameter<double> m_cb_thresh;
    };
}