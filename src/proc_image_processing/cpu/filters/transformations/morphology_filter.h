/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=MorphologyFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_MORPHOLOGY_H_
#define PROC_IMAGE_PROCESSING_FILTERS_MORPHOLOGY_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class MorphologyFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<MorphologyFilter>;

        explicit MorphologyFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  morph_type_("Morphology_type", 0, 0, 4, &parameters_,
                              "0=Gradient, 1=TopHat, 2=BlackHat, 3=Opening, 4=Closing"),
                  kernel_type_("Kernel_type", 0, 0, 2, &parameters_,
                               "0=Rect, 1=Elipse, 2=Cross"),
                  iteration_("Iteration", 1, 1, 20, &parameters_),
                  kernel_size_("Kernel_size", 1, 1, 40, &parameters_),
                  anchor_(-1, -1) {
            setName("MorphologyFilter");
        }

        virtual ~MorphologyFilter() {}

        virtual void apply(cv::Mat &image) {
            if (enable_()) {
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
                    case 0:
                        cv::morphologyEx(image, image, cv::MORPH_GRADIENT, kernel, anchor_,
                                         iteration_(), CV_8U);
                        break;
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
                }
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> morph_type_, kernel_type_, iteration_, kernel_size_;
        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_MORPHOLOGY_H_