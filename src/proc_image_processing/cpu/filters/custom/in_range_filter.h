// FACTORY_GENERATOR_CLASS_NAME=InRangeFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_IN_RANGE_FILTER_H_
#define PROC_IMAGE_PROCESSING_FILTERS_IN_RANGE_FILTER_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    /**
     * The filter inRange check if array elements lie between certain value of HSV
     * and Luv. If it does, value of pixel is set to = 1.
     * If not, value of pixel = 0.
     * In this case, this filter is a binarizer and will output a black and white
     * image
     */
    class InRangeFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<InRangeFilter>;

        explicit InRangeFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  lower_hue_("HSV Lower H", 0, 0, 255, &parameters_),
                  upper_hue_("HSV Upper H", 255, 0, 255, &parameters_),
                  lower_saturation_("HSV Lower S", 0, 0, 255, &parameters_),
                  upper_saturation_("HSV Upper S", 255, 0, 255, &parameters_),
                  lower_value_("HSV Lower V", 0, 0, 255, &parameters_),
                  upper_value_("HSV Upper V", 255, 0, 255, &parameters_),
                  lower_lightness_("LUV Lower L", 0, 0, 255, &parameters_),
                  upper_lightness_("LUV Upper L", 255, 0, 255, &parameters_),
                  lower_u_("LUV Lower U", 0, 0, 255, &parameters_),
                  upper_u_("LUV Upper U", 255, 0, 255, &parameters_),
                  lower_v_("LUV Lower V", 0, 0, 255, &parameters_),
                  upper_v_("LUV Upper V", 255, 0, 255, &parameters_) {
            setName("InRangeFilter");
        }

        ~InRangeFilter() override = default;

        /**
         * Overrides the execute function from the Filter class.
         * This is the function that is going to be called for processing the image.
         * This takes an image as a parameter and modify it with the filtered image.
         *
         * \param image The image to process.
         */
        void apply(cv::Mat &image) override {
            cv::Mat hsv;
            cv::Mat luv;

            cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV_FULL);
            cv::inRange(
                    hsv,
                    cv::Scalar(lower_hue_.getValue(), lower_saturation_.getValue(), lower_value_.getValue()),
                    cv::Scalar(upper_hue_.getValue(), upper_saturation_.getValue(), upper_value_.getValue()),
                    hsv
            );

            cv::cvtColor(image, luv, cv::COLOR_RGB2Luv);
            cv::inRange(
                    luv,
                    cv::Scalar(lower_lightness_.getValue(), lower_u_.getValue(), lower_v_.getValue()),
                    cv::Scalar(upper_lightness_.getValue(), upper_u_.getValue(), upper_v_.getValue()),
                    luv
            );
            cv::bitwise_and(hsv, luv, image);
        }

    private:
        /** Inclusive Hue lower boundary. */
        RangedParameter<int> lower_hue_;

        /**  Inclusive Hue upper boundary. */
        RangedParameter<int> upper_hue_;

        /** Inclusive Saturation lower boundary. */
        RangedParameter<int> lower_saturation_;

        /** Inclusive Saturation upper boundary. */
        RangedParameter<int> upper_saturation_;

        /** Inclusive Value lower boundary. */
        RangedParameter<int> lower_value_;

        /** Inclusive Value upper boundary. */
        RangedParameter<int> upper_value_;

        /** Inclusive Lightness lower boundary. */
        RangedParameter<int> lower_lightness_;

        /** Inclusive Lightness upper boundary. */
        RangedParameter<int> upper_lightness_;

        /** Inclusive u lower boundary. */
        RangedParameter<int> lower_u_;

        /** Inclusive u upper boundary. */
        RangedParameter<int> upper_u_;

        /** Inclusive v lower boundary. */
        RangedParameter<int> lower_v_;

        /** Inclusive v upper boundary. */
        RangedParameter<int> upper_v_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_IN_RANGE_FILTER_H_
