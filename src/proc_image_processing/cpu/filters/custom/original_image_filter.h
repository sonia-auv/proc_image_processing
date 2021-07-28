// FACTORY_GENERATOR_CLASS_NAME=OriginalImageFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_ORIGINAL_IMAGE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_ORIGINAL_IMAGE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class OriginalImageFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<OriginalImageFilter>;

        explicit OriginalImageFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams), enable_("Enable", false, &parameters_) {
            setName("OriginalImageFilter");
        }

        ~OriginalImageFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                image = global_param_handler_.getOriginalImage();
            }
        }

    private:
        Parameter<bool> enable_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_ORIGINAL_IMAGE_H_
