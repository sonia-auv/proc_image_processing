// FACTORY_GENERATOR_CLASS_NAME=RemoveMaskFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_REMOVE_MASK_H_
#define PROC_IMAGE_PROCESSING_FILTERS_REMOVE_MASK_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    // see http://docs.opencv.org/modules/imgproc/doc/filtering.html
    // for more detail on how and why
    //
    // This is a program to execute image filter other than erode, dilate and
    // morphologicalEx. Those are more blur function than pixelizer
    // settings are for the differents type of filters, and does not apply to all
    class RemoveMaskFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<RemoveMaskFilter>;

        explicit RemoveMaskFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  type_("Type", 2, 0, 3, &parameters_,
                        "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
                  kernel_size_("Kernel_size", 1, 0, 35, &parameters_),
                  anchor_(-1, -1) {
            setName("RemoveMaskFilter");
        }

        ~RemoveMaskFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                global_param_handler_.getOriginalImage().copyTo(image, image);
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> type_, kernel_size_;

        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_REMOVE_MASK_H_
