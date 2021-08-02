// FACTORY_GENERATOR_CLASS_NAME=EqualizeHistogramFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class EqualizeHistogramFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<EqualizeHistogramFilter>;

        explicit EqualizeHistogramFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("enable", false, &parameters_) {
            setName("EqualizeHistogramFilter");
        }

        ~EqualizeHistogramFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                cv::equalizeHist(image, image);
            }
        }

    private:
        Parameter<bool> enable_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_EQUALIZE_H_
