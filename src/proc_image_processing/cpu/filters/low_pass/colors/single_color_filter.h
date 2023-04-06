// FACTORY_GENERATOR_CLASS_NAME=BGRFilter

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class SingleColorFilter : public Filter
    {
    public:
        using Ptr = std::shared_ptr<SingleColorFilter>;

        explicit SingleColorFilter(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_color("Color", 0, 0, 2, &parameters_, "0=Blue, 1=Green, 2=Red")
        {
            setName("SingleColorFilter");
        }

        ~SingleColorFilter() override = default;

        void apply(cv::Mat &image) override
        {
            int blue = 1;
            int green = 1;
            int red = 1;
            switch (m_color())
            {
            case 0:
                green = 0;
                red = 0;
                break;

            case 1:
                blue = 0;
                red = 0;
                break;

            case 2:
                blue = 0;
                green = 0;
                break;
            
            default:
                break;
            }
            for(int r = 0; r < image.rows; ++r) {
                for(int c = 0; c < image.cols; ++c) {
                    image.at<cv::Vec3b>(r,c)[0] = image.at<cv::Vec3b>(r,c)[0] * blue;
                    image.at<cv::Vec3b>(r,c)[1] = image.at<cv::Vec3b>(r,c)[1] * green;
                    image.at<cv::Vec3b>(r,c)[2] = image.at<cv::Vec3b>(r,c)[2] * red;
                }
            }
        }
    private:
        RangedParameter<int> m_color;
    };
}