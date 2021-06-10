/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_ADAPTIVE_THRESHOLD_H_
#define PROVIDER_VISION_FILTERS_ADAPTIVE_THRESHOLD_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class AdaptiveThreshold : public Filter {
  public:
    using Ptr = std::shared_ptr<AdaptiveThreshold>;

    explicit AdaptiveThreshold(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("Enable", false, &parameters_),
      method_("Method", 0, 0, 1, &parameters_, "0=Gaussian 1=Mean"),
      threshold_type_("Threshold_type", 0, 0, 1, &parameters_,
        "0=BIN, 1=BIN_INV"),
      _block_size("Size", 1, 1, 40, &parameters_),
      c_param_("C_param", 0.0f, -255.0f, 255.0f, &parameters_) {
      SetName("AdaptiveThreshold");
    }

    virtual ~AdaptiveThreshold() {}

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        if (image.channels() > 1) {
          cv::cvtColor(image, image, CV_BGR2GRAY);
        }
        int size = _block_size() * 2 + 1;
        int method = method_() == 0 ? cv::ADAPTIVE_THRESH_GAUSSIAN_C
          : cv::ADAPTIVE_THRESH_MEAN_C;
        int type =
          threshold_type_() == 0 ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
        cv::adaptiveThreshold(image, image, 255, method, type, size, c_param_());
      }
    }

  private:
    Parameter<bool> enable_;

    RangedParameter<int> method_, threshold_type_, _block_size;

    RangedParameter<double> c_param_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_INRANGE_H_
