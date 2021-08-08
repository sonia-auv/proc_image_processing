// FACTORY_GENERATOR_CLASS_NAME=CropFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_IMAGE_CROPPER_H_
#define PROC_IMAGE_PROCESSING_FILTERS_IMAGE_CROPPER_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class CropFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<CropFilter>;

        explicit CropFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  x_offset_("X Offset", 0, 0, 2000, &parameters_),
                  y_offset_("Y Offset", 0, 0, 2000, &parameters_),
                  x_reduction_("X Reduction", 0, 0, 2000, &parameters_),
                  y_reduction_("Y Reduction", 0, 0, 2000, &parameters_) {
            setName("CropFilter");
        }

        ~CropFilter() override = default;

        void apply(cv::Mat &image) override {
            if ((x_offset_() + x_reduction_() < image.size[1]) |
                (y_offset_() + y_reduction_() < image.size[0])) {
                image = image(cv::Rect(x_offset_(), y_offset_(),
                                       image.size[1] - x_reduction_() - x_offset_(),
                                       image.size[0] - y_reduction_() - y_offset_()));
            }
        }

    private:
        RangedParameter<int> x_offset_, y_offset_, x_reduction_, y_reduction_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_IMAGE_CROPPER_H_
