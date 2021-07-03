/// \author	Pierluc Bédard <pierlucbed@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_IMAGE_CROPPER_H_
#define PROVIDER_VISION_FILTERS_IMAGE_CROPPER_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class ImageCropper : public Filter {
  public:
    using Ptr = std::shared_ptr<Blurr>;

    explicit ImageCropper(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      x_offset_("X Offset", 0, 0, 2000, &parameters_),
      y_offset_("Y Offset", 0, 0, 2000, &parameters_),
      x_reduction_("X Reduction", 0, 0, 2000, &parameters_),
      y_reduction_("Y Reduction", 0, 0, 2000, &parameters_) {
      SetName("ImageCropper");
    }

    virtual ~ImageCropper() {}

    virtual void ApplyFilter(cv::Mat& image) {
        if ((x_offset_() + x_reduction_() < image.size[1]) |
          (y_offset_() + y_reduction_() < image.size[0])) {
          image = image(cv::Rect(x_offset_(), y_offset_(),
            image.size[1] - x_reduction_() - x_offset_(),
            image.size[0] - y_reduction_() - y_offset_()));
        }
    }

  private:
    RangedParameter<int> x_offset_, y_offset_, x_reduction_, y_reduction_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_IMAGE_CROPPER_H_
