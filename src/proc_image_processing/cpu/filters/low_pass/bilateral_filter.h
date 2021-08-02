// FACTORY_GENERATOR_CLASS_NAME=BilateralFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_BILATERAL_FILTER_H_
#define PROC_IMAGE_PROCESSING_FILTERS_BILATERAL_FILTER_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    /**
     * The filter bilateral aims to blurr an image without loosing edge sharpness.
     * This act like a proxy for the OpenCv bilateralFilter method.
     */
    class BilateralFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<BilateralFilter>;

        explicit BilateralFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  diameter_("Diameter", -100, 0, 100, &parameters_),
                  sigma_color_("Sigma color", 0, 0, 300, &parameters_,
                               "Filter sigma in the color space. A larger value of the parameter means that "
                               "farther colors within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting "
                               "in larger areas of semi-equal color."),
                  sigma_space_("Sigma space", 0, 0, 300, &parameters_,
                               "Filter sigma in the coordinate space. A larger value of the parameter means that "
                               "farther pixels will influence each other as long as their colors are close enough (see sigmaColor "
                               "). When d>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is "
                               "proportional to sigmaSpace.") {
            setName("BilateralFilter");
        }

        ~BilateralFilter() override = default;

        /**
         * Override the execute function from the Filter class.
         * This is the function that is going to be called for processing the image.
         * This takes an image as a parameter and modify it with the filtered image.
         *
         * \param image The image to process.
         */
        void apply(cv::Mat &image) override {
            if (enable_()) {
                cv::Mat blurred;
                cv::bilateralFilter(
                        image,
                        blurred,
                        diameter_.getValue(),
                        sigma_color_.getValue(),
                        sigma_space_.getValue()
                );

                blurred.copyTo(image);
            }
        }

    private:
        /**
         * State if the filter is enabled or not.
         * This is being used by the vision server for calling the filter in the
         * filterchain.
         */
        Parameter<bool> enable_;

        /**
         * From the OpenCV definition:
         * Diameter of each pixel neighborhood that is used during filtering.
         * If it is non-positive, it is computed from sigmaSpace .
         */
        RangedParameter<int> diameter_;

        /**
         * From the OpenCV definition:
         * Filter sigma in the color space. A larger value of the parameter means
         * that farther colors within the pixel neighborhood (see sigmaSpace )
         * will be mixed together, resulting in larger areas of semi-equal color.
         */
        RangedParameter<int> sigma_color_;

        /**
         * From the OpenCV definition:
         * Filter sigma in the coordinate space. A larger value of the parameter
         * means that farther pixels will influence each other as long as their
         * colors are close enough (see sigmaColor ). When d>0 , it specifies
         * the neighborhood size regardless of sigmaSpace . Otherwise, d is
         * proportional to sigmaSpace .
         */
        RangedParameter<int> sigma_space_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_BILATERAL_FILTER_H_
