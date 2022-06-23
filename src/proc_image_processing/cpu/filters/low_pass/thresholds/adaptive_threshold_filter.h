// FACTORY_GENERATOR_CLASS_NAME=AdaptiveThresholdFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_ADAPTIVE_THRESHOLD_H_
#define PROC_IMAGE_PROCESSING_FILTERS_ADAPTIVE_THRESHOLD_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class AdaptiveThresholdFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<AdaptiveThresholdFilter>;

        explicit AdaptiveThresholdFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  method_("Method : false=Gaussian true=Mean", false, &parameters_),
                  threshold_type_("Threshold type : false=BIN, true=BIN_INV", false, &parameters_),
                  _block_size("Size", 1, 1, 40, &parameters_),
                  c_("C", 0.0f, -255.0f, 255.0f, &parameters_,
                     "Constant subtracted from the mean or weighted mean.") {
            setName("AdaptiveThresholdFilter");
        }

        ~AdaptiveThresholdFilter() override = default;

        void apply(cv::Mat &image) override {
            if (image.channels() > 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }
            int size = _block_size() * 2 + 1;
            int method = method_() ? cv::ADAPTIVE_THRESH_MEAN_C : cv::ADAPTIVE_THRESH_GAUSSIAN_C ;
            int type = threshold_type_() ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
            cv::adaptiveThreshold(image, image, 255, method, type, size, c_());
        }

    private:
        Parameter<bool> method_;
        Parameter<bool> threshold_type_;
        RangedParameter<int> _block_size;
        RangedParameter<double> c_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_ADAPTIVE_THRESHOLD_H_
