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
              m_blue("Blue", 40.0, 0.0, 255.0, &parameters_, "How much Blue"),
              m_green("Green", 40.0, 0.0, 255.0, &parameters_, "How much Green"),
              m_red("Red", 40.0, 0.0, 255.0, &parameters_, "How much Red")
        {
            setName("BGRFilter");
        }

        ~BGRFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::Vec3b bgrPixel(m_blue(), m_green(), m_red());
            
            int thresh = 40;

            cv::Scalar minBGR = cv::Scalar(bgrPixel.val[0] - thresh, bgrPixel.val[1] - thresh, bgrPixel.val[2] - thresh);
            cv::Scalar maxBGR = cv::Scalar(bgrPixel.val[0] + thresh, bgrPixel.val[1] + thresh, bgrPixel.val[2] + thresh);
 
            cv::Mat maskBGR, resultBGR;
            cv::inRange(image, minBGR, maxBGR, maskBGR);
            cv::bitwise_and(image, image, resultBGR, maskBGR);
            resultBGR.copyTo(image);
        }

    private:
        RangedParameter<double> m_blue;
        RangedParameter<double> m_green;
        RangedParameter<double> m_red;
    };
}