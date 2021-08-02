// FACTORY_GENERATOR_CLASS_NAME=ThresholdFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_THRESHOLD_H_
#define PROC_IMAGE_PROCESSING_FILTERS_THRESHOLD_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class ThresholdFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ThresholdFilter>;

        explicit ThresholdFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  type_("Threshold type", 1, 0, 5, &parameters_,
                        "0=BIN, 1=BIN_INV, 2=TRUNC, 3=TOZERO, 4=TOZERO_INV 5=OTSU"),
                  max_("Maximum value", 100, 0, 255, &parameters_) {
            setName("ThresholdFilter");
        }

        ~ThresholdFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                if (image.channels() > 1) {
                    cv::cvtColor(image, image, CV_BGR2GRAY);
                }
                if (image.depth() != CV_8U) {
                    image.convertTo(image, CV_8U);
                }

                int threshold_type;
                switch (type_()) {
                    case 0:
                        threshold_type = CV_THRESH_BINARY;
                        break;
                    case 1:
                        threshold_type = CV_THRESH_BINARY_INV;
                        break;
                    case 2:
                        threshold_type = CV_THRESH_TRUNC;
                        break;
                    case 3:
                        threshold_type = CV_THRESH_TOZERO;
                        break;
                    case 4:
                        threshold_type = CV_THRESH_TOZERO_INV;
                        break;
                    case 5:
                        threshold_type = CV_THRESH_OTSU;
                        break;
                    default:
                        threshold_type = CV_THRESH_BINARY;
                        break;
                }
                cv::threshold(image, image, max_(), 255, threshold_type);
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> type_, max_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_THRESHOLD_H_
