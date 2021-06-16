/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FILTER_GENERATOR_CLASS_NAME=Rotate

#ifndef PROVIDER_VISION_FILTERS_ROTATE_H_
#define PROVIDER_VISION_FILTERS_ROTATE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class Rotate : public Filter {
  public:
    using Ptr = std::shared_ptr<Rotate>;

    explicit Rotate(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("enable", false, &parameters_),
      transpose_("transpose", false, &parameters_),
      rotate_type_("Rotation_type", 0, 0, 3, &parameters_,
        "Rotate type: 0=NONE, 1=x axis, 2=y axis, 3=all axis") {
      SetName("Rotate");
    }

    virtual ~Rotate() {}

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        if (transpose_()) cv::transpose(image, image);
        switch (rotate_type_()) {
        case 0:
          break;
        case 1:
          cv::flip(image, image, 0);
          break;
        case 2:
          cv::flip(image, image, 1);
          break;
        case 3:
          cv::flip(image, image, -1);
          break;
        }
      }
    }

  private:
    Parameter<bool> enable_, transpose_;

    RangedParameter<int> rotate_type_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_ROTATE_H_
