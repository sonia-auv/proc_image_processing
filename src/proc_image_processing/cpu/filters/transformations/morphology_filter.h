// FACTORY_GENERATOR_CLASS_NAME=MorphologyFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_MORPHOLOGY_H_
#define PROC_IMAGE_PROCESSING_FILTERS_MORPHOLOGY_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class MorphologyFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<MorphologyFilter>;

        explicit MorphologyFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  morph_type_("Morphology type", 0, 0, 4, &parameters_,
                              "0=Gradient, 1=TopHat, 2=BlackHat, 3=Opening, 4=Closing"),
                  kernel_type_("Kernel type", 0, 0, 2, &parameters_, "0=Rect, 1=Elipse, 2=Cross"),
                  iteration_("Iteration", 1, 1, 20, &parameters_),
                  kernel_size_("Kernel size", 1, 1, 40, &parameters_),
                  anchor_(-1, -1) {
            setName("MorphologyFilter");
        }

        ~MorphologyFilter() override = default;

        void apply(cv::Mat &image) override {
            if (image.channels() > 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }

            // Kernel selection
            int kernelType;
            switch (kernel_type_()) {
                case 0:
                    kernelType = cv::MORPH_RECT;
                    break;
                case 1:
                    kernelType = cv::MORPH_ELLIPSE;
                    break;
                case 2:
                    kernelType = cv::MORPH_CROSS;
                    break;
                default:
                    kernelType = cv::MORPH_RECT;
                    break;
            }

            // Creating the kernel
            cv::Mat kernel = cv::getStructuringElement(
                    kernelType, cv::Size(kernel_size_() * 2 + 1, kernel_size_() * 2 + 1),
                    anchor_);

            // Selecting with _morph_type wich operation to use
            switch (morph_type_()) {
                case 1:
                    cv::morphologyEx(image, image, cv::MORPH_TOPHAT, kernel, anchor_,
                                     iteration_(), CV_8U);
                    break;
                case 2:
                    cv::morphologyEx(image, image, cv::MORPH_BLACKHAT, kernel, anchor_,
                                     iteration_(), CV_8U);
                    break;
                case 3:
                    cv::morphologyEx(image, image, cv::MORPH_OPEN, kernel, anchor_,
                                     iteration_(), CV_8U);
                    break;
                case 4:
                    cv::morphologyEx(image, image, cv::MORPH_CLOSE, kernel, anchor_,
                                     iteration_(), CV_8U);
                    break;
                default:
                    cv::morphologyEx(image, image, cv::MORPH_GRADIENT, kernel, anchor_, iteration_(), CV_8U);
                    break;
            }
        }

    private:
        RangedParameter<int> morph_type_;
        RangedParameter<int> kernel_type_;
        RangedParameter<int> iteration_;
        RangedParameter<int> kernel_size_;

        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_MORPHOLOGY_H_
