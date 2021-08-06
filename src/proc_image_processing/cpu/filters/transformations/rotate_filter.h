// FACTORY_GENERATOR_CLASS_NAME=RotateFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_ROTATE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_ROTATE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class RotateFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<RotateFilter>;

        explicit RotateFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  transpose_("transpose", false, &parameters_),
                  rotate_type_("Rotation_type", 0, 0, 3, &parameters_,
                               "RotateFilter type: 0=NONE, 1=x axis, 2=y axis, 3=all axis") {
            setName("RotateFilter");
        }

        ~RotateFilter() override = default;

        void apply(cv::Mat &image) override {
            if (transpose_()) cv::transpose(image, image);
            switch (rotate_type_()) {
                case 1:
                    cv::flip(image, image, 0);
                    break;
                case 2:
                    cv::flip(image, image, 1);
                    break;
                case 3:
                    cv::flip(image, image, -1);
                    break;
                default:
                    break;
            }
        }

    private:
        Parameter<bool> transpose_;

        RangedParameter<int> rotate_type_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_ROTATE_H_
