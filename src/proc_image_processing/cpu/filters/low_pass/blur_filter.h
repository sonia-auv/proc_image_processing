// FACTORY_GENERATOR_CLASS_NAME=BlurFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_BLUR_H_
#define PROC_IMAGE_PROCESSING_FILTERS_BLUR_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    // see http://docs.opencv.org/modules/imgproc/doc/filtering.html
    // for more detail on how and why
    //
    // This is a program to execute image filter other than erode, dilate and
    // morphologicalEx. Those are more blur function than pixelizer
    // settings are for the differents type of filters, and does not apply to all
    class BlurFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<BlurFilter>;

        explicit BlurFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  type_("Type", 2, 0, 3, &parameters_,
                        "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
                  kernel_size_("Kernel_size", 1, 0, 35, &parameters_),
                  anchor_(-1, -1) {
            setName("BlurFilter");
        }

        ~BlurFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::Size2i kernelSize(kernel_size_() * 2 + 1,
                                  kernel_size_() * 2 + 1);
            switch (type_()) {
                // Could be optimized via function pointer maybe?
                case 1:
                    cv::blur(image, image, kernelSize, anchor_);
                    break;
                case 2:
                    cv::GaussianBlur(image, image, kernelSize, 0, 0);
                    break;
                case 3:
                    cv::medianBlur(image, image, kernel_size_() * 2 + 1);
                    break;
                default:
                    break;
            }
        }

    private:
        RangedParameter<int> type_, kernel_size_;

        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_BLUR_H_
