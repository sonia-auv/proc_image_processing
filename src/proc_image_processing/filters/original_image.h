/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=OriginalImage

#ifndef PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_
#define PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class OriginalImage : public Filter {
  public:
    using Ptr = std::shared_ptr<OriginalImage>;

    explicit OriginalImage(const GlobalParamHandler& globalParams)
      : Filter(globalParams) {
      SetName("OriginalImage");
    }

    virtual ~OriginalImage() {}

    void Apply(cv::Mat& image) override {
        image = global_params_.getOriginalImage();
    }
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_ORIGINAL_IMAGE_H_
