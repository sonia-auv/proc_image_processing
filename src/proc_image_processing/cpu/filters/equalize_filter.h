/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=Equalize

#ifndef PROVIDER_VISION_FILTERS_EQUALIZE_H_
#define PROVIDER_VISION_FILTERS_EQUALIZE_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

  class Equalize : public Filter {
  public:
    using Ptr = std::shared_ptr<Equalize>;

    explicit Equalize(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("enable", false, &parameters_) {
      SetName("Equalize");
    }

    virtual ~Equalize() {}

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        cv::equalizeHist(image, image);
      }
    }

  private:
    Parameter<bool> enable_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_EQUALIZE_H_
