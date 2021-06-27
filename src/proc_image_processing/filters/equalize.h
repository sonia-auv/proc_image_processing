/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_EQUALIZE_H_
#define PROVIDER_VISION_FILTERS_EQUALIZE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class Equalize : public IFilter {
  public:
    using Ptr = std::shared_ptr<Equalize>;

    explicit Equalize(const GlobalParamHandler& globalParams)
      : IFilter(globalParams),
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
