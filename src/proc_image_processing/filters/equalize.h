/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=Equalize

#ifndef PROVIDER_VISION_FILTERS_EQUALIZE_H_
#define PROVIDER_VISION_FILTERS_EQUALIZE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class Equalize : public Filter {
  public:
    using Ptr = std::shared_ptr<Equalize>;

    explicit Equalize(const GlobalParamHandler& globalParams)
      : Filter(globalParams) {
      SetName("Equalize");
    }

    virtual ~Equalize() {}

    virtual void ApplyFilter(cv::Mat& image) {
        cv::equalizeHist(image, image);
    }
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_EQUALIZE_H_
