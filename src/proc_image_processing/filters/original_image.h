/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_
#define PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class OriginalImage : public AbstractFilter {
  public:
    using Ptr = std::shared_ptr<OriginalImage>;

    explicit OriginalImage(const GlobalParamHandler& globalParams)
      : AbstractFilter(globalParams), enable_("Enable", false, &parameters_) {
      SetName("OriginalImage");
    }

    virtual ~OriginalImage() {}

    virtual void ProcessImage(cv::Mat& image) {

        image = global_params_.getOriginalImage();
      }
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_
