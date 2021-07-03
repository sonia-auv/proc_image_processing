/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_
#define PROVIDER_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class SubmarineFrameMasker : public Filter {
  public:
    using Ptr = std::shared_ptr<SubmarineFrameMasker>;

    explicit SubmarineFrameMasker(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("Enable", false, &parameters_),
      rotate_type_("Rotation_type", 0, 0, 3, &parameters_,
        "Rotate type: 0=NONE, 1=x axis, 2=y axis, 3=all axis"),
      prev_rot_value_(0) {
      SetName("SubmarineFrameMasker");
      std::string mask_name =
        std::string(getenv("SONIA_WORKSPACE_ROOT")) +
        std::string("/ros/src/vision_server/config/bottom_mask.jpg");
      bottom_mask_ = cv::imread(mask_name, CV_LOAD_IMAGE_GRAYSCALE);
    }

    virtual ~SubmarineFrameMasker() {}

    virtual void ApplyFilter(cv::Mat& image) {
      if (enable_()) {
        if (prev_rot_value_ != rotate_type_()) {
          prev_rot_value_ = rotate_type_();
          switch (rotate_type_()) {
          case 1:
            cv::flip(bottom_mask_, bottom_mask_, 0);
            break;
          case 2:
            cv::flip(bottom_mask_, bottom_mask_, 1);
            break;
          case 3:
            cv::flip(bottom_mask_, bottom_mask_, -1);
            break;
          }
        }
        if (image.size() == bottom_mask_.size())
          cv::bitwise_and(image, bottom_mask_, image);
      }
    }

  private:
    Parameter<bool> enable_;
    RangedParameter<int> rotate_type_;
    cv::Mat bottom_mask_;
    int prev_rot_value_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_
