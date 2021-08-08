// FACTORY_GENERATOR_CLASS_NAME=OriginalImageFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_ORIGINAL_IMAGE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_ORIGINAL_IMAGE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class OriginalImageFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<OriginalImageFilter>;

        explicit OriginalImageFilter(const GlobalParameterHandler &globalParams): Filter(globalParams) {
            setName("OriginalImageFilter");
        }

        ~OriginalImageFilter() override = default;

        void apply(cv::Mat &image) override {
            image = global_param_handler_.getOriginalImage();
        }
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_ORIGINAL_IMAGE_H_
