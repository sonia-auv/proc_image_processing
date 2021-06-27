/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_
#define PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class OriginalImage : public IFilter {
  public:
    using Ptr = std::shared_ptr<OriginalImage>;

    explicit OriginalImage(const GlobalParamHandler& globalParams)
      : IFilter(globalParams), enable_("Enable", false, &parameters_) {
      SetName("OriginalImage");
    }

    virtual ~OriginalImage() {}

    virtual void ProcessImage(cv::Mat& image) {
      if (enable_()) {
        image = global_params_.getOriginalImage();
      }
    }

  private:
    Parameter<bool> enable_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_
