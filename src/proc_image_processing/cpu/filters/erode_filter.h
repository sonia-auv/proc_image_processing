/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=Erode

#ifndef PROVIDER_VISION_FILTERS_ERODE_H_
#define PROVIDER_VISION_FILTERS_ERODE_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

  class Erode : public Filter {
  public:
    using Ptr = std::shared_ptr<Erode>;

    explicit Erode(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("Enable", false, &parameters_),
      use_square_kernel_("Square_kernel", true, &parameters_),
      kernel_type_("Kernel_type", 0, 0, 2, &parameters_),
      kernel_size_x_("Width", 1, 0, 20, &parameters_),
      kernel_size_y_("Height", 1, 0, 20, &parameters_),
      iteration_("Iteration", 1, 0, 20, &parameters_),
      anchor_(-1, -1) {
        setName("Erode");
    }

    virtual ~Erode() {}

      virtual void apply(cv::Mat &image) {
          if (enable_()) {
              int kernel_type = 0;
              switch (kernel_type_()) {
                  case 0:
                      kernel_type = cv::MORPH_RECT;
                      break;
                  case 1:
                      kernel_type = cv::MORPH_ELLIPSE;
                      break;
                  case 2:
          kernel_type = cv::MORPH_CROSS;
          break;
        }

        cv::Size size(kernel_size_x_() * 2 + 1,
          (use_square_kernel_() ? kernel_size_x_() * 2 + 1
            : kernel_size_y_() * 2 + 1));
        cv::Mat kernel = cv::getStructuringElement(kernel_type, size, anchor_);

        cv::erode(image, image, kernel, anchor_, iteration_());
      }
    }

  private:
    Parameter<bool> enable_, use_square_kernel_;
    RangedParameter<int> kernel_type_;
    RangedParameter<int> kernel_size_x_, kernel_size_y_;
    RangedParameter<int> iteration_;

    const cv::Point anchor_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_ERODE_H_
