/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_CANNY_H_
#define PROVIDER_VISION_FILTERS_CANNY_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

  class Canny : public Filter {
  public:
    using Ptr = std::shared_ptr<Canny>;

    explicit Canny(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("Enable", false, &parameters_),
      l2_gradiant_("l2_gradient", false, &parameters_),
      thresh_one_("thres_one", 100, 0, 255, &parameters_),
      thresh_two_("thres_two", 200, 0, 255, &parameters_),
      aperture_size_("Aperture_size", 3, 0, 20, &parameters_) {
      SetName("Canny");
    }

    virtual ~Canny() {}

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        if (image.channels() > 1) {
          cv::cvtColor(image, image, CV_BGR2GRAY);
        }
        cv::Canny(image, image, thresh_one_(), thresh_two_(),
          aperture_size_() * 2 + 1, l2_gradiant_());
      }
    }

  private:
    Parameter<bool> enable_, l2_gradiant_;
    RangedParameter<int> thresh_one_, thresh_two_, aperture_size_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SOBEL_H_
