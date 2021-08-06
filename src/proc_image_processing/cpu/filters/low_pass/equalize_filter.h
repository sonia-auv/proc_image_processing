// FACTORY_GENERATOR_CLASS_NAME=EqualizeFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class EqualizeFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<EqualizeFilter>;

        explicit EqualizeFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams) {
            setName("EqualizeFilter");
        }

        ~EqualizeFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::equalizeHist(image, image);
        }
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_
