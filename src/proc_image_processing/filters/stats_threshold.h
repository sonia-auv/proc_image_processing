/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=StatsThreshold

#ifndef PROVIDER_VISION_FILTERS_STATS_THRESHOLD_H_
#define PROVIDER_VISION_FILTERS_STATS_THRESHOLD_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class StatsThreshold : public Filter {
  public:
    using Ptr = std::shared_ptr<StatsThreshold>;

    explicit StatsThreshold(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      min_thresh_("Min_thresh", 0, 0, 255, &parameters_),
      mean_multiplier_("Mean_multiplier", 1, -10, 10, &parameters_),
      std_dev_multiplier_("Standard_deviation_multiplier", 1, -10, 10,
        &parameters_) {
      SetName("StatsThreshold");
    }

    virtual ~StatsThreshold() {}

    void Apply(cv::Mat& image) override {
        if (image.channels() > 1) {
          cv::cvtColor(image, image, CV_BGR2GRAY);
        }
        if (image.depth() != CV_8U) {
          image.convertTo(image, CV_8U);
        }
        cv::Scalar mean, stdDev;

        cv::meanStdDev(image, mean, stdDev);
        int thresh_val =
          mean[0] * mean_multiplier_() + stdDev[0] * std_dev_multiplier_();
        thresh_val = thresh_val < min_thresh_() ? min_thresh_() : thresh_val;
        cv::threshold(image, image, thresh_val, 255, CV_THRESH_BINARY);
    }

  private:
    RangedParameter<int> min_thresh_;
    RangedParameter<double> mean_multiplier_, std_dev_multiplier_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_STATS_THRESHOLD_H_
