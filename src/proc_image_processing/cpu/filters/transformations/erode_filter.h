// FACTORY_GENERATOR_CLASS_NAME=ErodeFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_ERODE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_ERODE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class ErodeFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ErodeFilter>;

        explicit ErodeFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  use_square_kernel_("Square kernel", true, &parameters_),
                  kernel_type_("Kernel type", 0, 0, 2, &parameters_),
                  kernel_size_x_("Width", 1, 0, 20, &parameters_),
                  kernel_size_y_("Height", 1, 0, 20, &parameters_),
                  iteration_("Iteration", 1, 0, 20, &parameters_),
                  anchor_(-1, -1) {
            setName("ErodeFilter");
        }

        ~ErodeFilter() override = default;

        void apply(cv::Mat &image) override {
            int kernel_type = 0;
            switch (kernel_type_()) {
                case 1:
                    kernel_type = cv::MORPH_ELLIPSE;
                    break;
                case 2:
                    kernel_type = cv::MORPH_CROSS;
                    break;
                default:
                    kernel_type = cv::MORPH_RECT;
                    break;
            }

            cv::Size size(
                    kernel_size_x_() * 2 + 1,
                    (use_square_kernel_() ? kernel_size_x_() * 2 + 1 : kernel_size_y_() * 2 + 1)
            );
            cv::Mat kernel = cv::getStructuringElement(kernel_type, size, anchor_);

            cv::erode(image, image, kernel, anchor_, iteration_());
        }

    private:
        Parameter<bool> use_square_kernel_;
        RangedParameter<int> kernel_type_;
        RangedParameter<int> kernel_size_x_;
        RangedParameter<int> kernel_size_y_;
        RangedParameter<int> iteration_;
        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_ERODE_H_
