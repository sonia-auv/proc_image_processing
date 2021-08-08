// FACTORY_GENERATOR_CLASS_NAME=BackgroundSubtractFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_BACKGROUND_SUBSTRACT_H_
#define PROC_IMAGE_PROCESSING_FILTERS_BACKGROUND_SUBSTRACT_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    // see http://docs.opencv.org/modules/imgproc/doc/filtering.html
    // for more detail on how and why
    //
    // This is a program to execute image filter other than erode, dilate and
    // morphologicalEx. Those are more blur function than pixelizer
    // settings are for the differents type of filters, and does not apply to all
    class BackgroundSubtractFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<BackgroundSubtractFilter>;

        explicit BackgroundSubtractFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  show_blurred_("Show blurred", false, &parameters_),
                  blur_size_("Blur size", 255, 0, 1000, &parameters_) {
            setName("BackgroundSubtractFilter");
        }

        ~BackgroundSubtractFilter() override = default;

        void apply(cv::Mat &image) override {
            std::vector<cv::Mat> channels;
            split(image, channels);
            cv::Mat b = channels[0];
            cv::Mat g = channels[1];
            cv::Mat r = channels[2];
            cv::Mat blurB, blurG, blurR;
            cv::blur(b, blurB, cv::Size(blur_size_.getValue(), blur_size_.getValue()));
            cv::blur(g, blurG, cv::Size(blur_size_.getValue(), blur_size_.getValue()));
            cv::blur(r, blurR, cv::Size(blur_size_.getValue(), blur_size_.getValue()));
            if (show_blurred_()) {
                b = blurB;
                g = blurG;
                r = blurR;
            } else {
                b = b - blurB;
                g = g - blurG;
                r = r - blurR;
            }
            channels[0] = b;
            channels[1] = g;
            channels[2] = r;
            cv::merge(channels, image);
        }

    private:
        Parameter<bool> show_blurred_;
        RangedParameter<int> blur_size_;
        RangedParameter<int> sigma_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_BACKGROUND_SUBSTRACT_H_
