// FACTORY_GENERATOR_CLASS_NAME=ContrastAndBrightnessFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {
    // Filter showing planes of different analysis (gray, _hsi, _bgr)
    // No threshold
    class ContrastAndBrightnessFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ContrastAndBrightnessFilter>;

        explicit ContrastAndBrightnessFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  contrast_("Contrast", 0, 0, 256, &parameters_, "Contrast"),
                  brightness_("Brightness", 0, -256, 256, &parameters_, "Set Brightness") {
            setName("ContrastAndBrightnessFilter");
        }

        ~ContrastAndBrightnessFilter() override = default;

        void apply(cv::Mat &image) override {
            image.convertTo(image, -1, contrast_(), brightness_());
        }

    private:
        RangedParameter<double> contrast_;
        RangedParameter<double> brightness_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
