// FACTORY_GENERATOR_CLASS_NAME=LaplacianFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_LAPLACIAN_H_
#define PROC_IMAGE_PROCESSING_FILTERS_LAPLACIAN_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class LaplacianFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<LaplacianFilter>;

        explicit LaplacianFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  convert_to_uchar_("Convert_to_uchar", true, &parameters_),
                  delta_("Delta", 0, 0, 255, &parameters_),
                  scale_("Scale", 1, 0, 255, &parameters_),
                  size_("Size", 2, 1, 20, &parameters_) {
            setName("LaplacianFilter");
        }

        ~LaplacianFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                if (image.channels() > 1) {
                    cv::cvtColor(image, image, CV_BGR2GRAY);
                }
                int size = size_() * 2 + 1;

                if (convert_to_uchar_()) {
                    cv::Laplacian(image, image, CV_8U, size, scale_(), delta_(),
                                  cv::BORDER_DEFAULT);
                } else {
                    cv::Laplacian(image, image, CV_32F, size, scale_(), delta_(),
                                  cv::BORDER_DEFAULT);
                }
            }
        }

    private:
        Parameter<bool> enable_, convert_to_uchar_;
        RangedParameter<double> delta_, scale_;
        RangedParameter<int> size_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_LAPLACIAN_H_
