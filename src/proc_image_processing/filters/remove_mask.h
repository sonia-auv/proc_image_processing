/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=RemoveMask

#ifndef PROVIDER_VISION_FILTERS_REMOVE_MASK_H_
#define PROVIDER_VISION_FILTERS_REMOVE_MASK_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  // see http://docs.opencv.org/modules/imgproc/doc/filtering.html
  // for more detail on how and why
  //
  // This is a program to execute image filter other than erode, dilate and
  // morphologicalEx. Those are more blur function than pixelizer
  // settings are for the differents type of filters, and does not apply to all
  class RemoveMask : public Filter {
  public:
    using Ptr = std::shared_ptr<RemoveMask>;

    explicit RemoveMask(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      type_("Type", 2, 0, 3, &parameters_,
        "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
      kernel_size_("Kernel_size", 1, 0, 35, &parameters_),
      anchor_(-1, -1) {
      setName("RemoveMask");
    }

    virtual ~RemoveMask() {}

    void apply(cv::Mat& image) override {
        global_params_.getOriginalImage().copyTo(image, image);
    }

  private:
    RangedParameter<int> type_, kernel_size_;

    const cv::Point anchor_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_REMOVE_MASK_H_