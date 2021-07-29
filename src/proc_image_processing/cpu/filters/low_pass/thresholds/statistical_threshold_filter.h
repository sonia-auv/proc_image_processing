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
                  min_thresh_("Min_thresh", 0, 0, 255, &parameters_),
                  mean_multiplier_("Mean_multiplier", 1, -10, 10, &parameters_),
                  std_dev_multiplier_("Standard_deviation_multiplier", 1, -10, 10,
                                      &parameters_) {
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
                int thresh_val = mean[0] * mean_multiplier_() + stdDev[0] * std_dev_multiplier_();
                thresh_val = thresh_val < min_thresh_() ? min_thresh_() : thresh_val;
                cv::threshold(image, image, thresh_val, 255, CV_THRESH_BINARY);
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> min_thresh_;
        RangedParameter<double> mean_multiplier_, std_dev_multiplier_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_STATS_THRESHOLD_H_
