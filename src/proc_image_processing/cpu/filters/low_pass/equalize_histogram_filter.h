// FACTORY_GENERATOR_CLASS_NAME=EqualizeHistogramFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class EqualizeHistogramFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<EqualizeHistogramFilter>;

        explicit EqualizeHistogramFilter(const GlobalParameterHandler &globalParams) : Filter(globalParams) {
            setName("EqualizeHistogramFilter");
        }

        ~EqualizeHistogramFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::equalizeHist(image, image);
        }
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_
