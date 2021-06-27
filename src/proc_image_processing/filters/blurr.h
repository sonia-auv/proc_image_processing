/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_BLURR_H_
#define PROVIDER_VISION_FILTERS_BLURR_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  // see http://docs.opencv.org/modules/imgproc/doc/filtering.html
  // for more detail on how and why
  //
  // This is a program to execute image filter other than erode, dilate and
  // morphologicalEx. Those are more blur function than pixelizer
  // settings are for the differents type of filters, and does not apply to all
  class Blurr : public AbstractFilter {
  public:
    using Ptr = std::shared_ptr<Blurr>;

    explicit Blurr(const GlobalParamHandler& globalParams)
      : AbstractFilter(globalParams),
      enable_("Enable", false, &parameters_),
      type_("Type", 2, 0, 3, &parameters_,
        "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
      kernel_size_("Kernel_size", 1, 0, 35, &parameters_),
      anchor_(-1, -1) {
      SetName("Blurr");
    }

    virtual ~Blurr() {}

    virtual void ProcessImage(cv::Mat& image) {
        cv::Size2i kernelSize((int)kernel_size_() * 2 + 1,
          (int)(kernel_size_() * 2 + 1));
        switch (type_()) {
          // Could be optimized via function pointer maybe?
        case 0:
          break;
        case 1:
          cv::blur(image, image, kernelSize, anchor_);
          break;
        case 2:
          cv::GaussianBlur(image, image, kernelSize, 0, 0);
          break;
        case 3:
          cv::medianBlur(image, image, kernel_size_() * 2 + 1);
          break;
        }
    }

  private:
    RangedParameter<int> type_, kernel_size_;

    const cv::Point anchor_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_BLURR_H_
