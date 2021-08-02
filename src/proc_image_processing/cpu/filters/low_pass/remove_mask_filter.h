// FACTORY_GENERATOR_CLASS_NAME=RemoveMaskFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_REMOVE_MASK_H_
#define PROC_IMAGE_PROCESSING_FILTERS_REMOVE_MASK_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class RemoveMaskFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<RemoveMaskFilter>;

        explicit RemoveMaskFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_) {
            setName("RemoveMaskFilter");
        }

        ~RemoveMaskFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                // TODO isn't this the same as Original image filter?
                global_param_handler_.getOriginalImage().copyTo(image, image);
            }
        }

    private:
        Parameter<bool> enable_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_REMOVE_MASK_H_
