// FACTORY_GENERATOR_CLASS_NAME=StatisticalThresholdFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_STATS_THRESHOLD_H_
#define PROC_IMAGE_PROCESSING_FILTERS_STATS_THRESHOLD_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class StatisticalThresholdFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<StatisticalThresholdFilter>;

        explicit StatisticalThresholdFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  min_thresh_("Minimum threshold", 0, 0, 255, &parameters_),
                  mean_factor_("Mean factor", 1, -10, 10, &parameters_),
                  std_dev_factor_("Standard deviation factor", 1, -10, 10, &parameters_) {
            setName("StatisticalThresholdFilter");
        }

        ~StatisticalThresholdFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                if (image.channels() > 1) {
                    cv::cvtColor(image, image, CV_BGR2GRAY);
                }
                if (image.depth() != CV_8U) {
                    image.convertTo(image, CV_8U);
                }
                cv::Scalar mean, stdDev;

                cv::meanStdDev(image, mean, stdDev);
                int thresh_val = mean[0] * mean_factor_() + stdDev[0] * std_dev_factor_();
                thresh_val = thresh_val < min_thresh_() ? min_thresh_() : thresh_val;
                cv::threshold(image, image, thresh_val, 255, CV_THRESH_BINARY);
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> min_thresh_;
        RangedParameter<double> mean_factor_, std_dev_factor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_STATS_THRESHOLD_H_
