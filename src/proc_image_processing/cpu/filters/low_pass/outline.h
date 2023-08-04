// FACTORY_GENERATOR_CLASS_NAME=Outline

#pragma once

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing
{
    class Outline : public Filter
    {
    public:
        using Ptr = std::shared_ptr<Outline>;

        explicit Outline(const GlobalParameterHandler &globalParams)
            : Filter(globalParams),
              m_thick("Thickness", 1, 1, 100, &parameters_, "The thickness of the outline"),
              m_blue("Blue", 255, 0, 255, &parameters_, "The amount of blue in the outline"),
              m_green("Green", 255, 0, 255, &parameters_, "The amount of green in the outline"),
              m_red("Red", 255, 0, 255, &parameters_, "The amount of red in the outline")
        {
            setName("Outline");
        }

        ~Outline() override = default;

        /**
         * Apply the filter
         * 
         * @param image The image to apply the changes to.
        */
        void apply(cv::Mat &image) override
        {
            cv::Rect rect(0, 0, image.cols, image.rows);

            cv::rectangle(image, rect, cv::Scalar(m_blue(), m_green(), m_red()), m_thick());
        }

    private:
        RangedParameter<int> m_thick;
        RangedParameter<int> m_blue;
        RangedParameter<int> m_green;
        RangedParameter<int> m_red;
    };
}