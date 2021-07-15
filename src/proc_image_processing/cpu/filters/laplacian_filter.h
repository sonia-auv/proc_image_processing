/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=LaplacianFilter

#ifndef PROVIDER_VISION_FILTERS_LAPLACIAN_H_
#define PROVIDER_VISION_FILTERS_LAPLACIAN_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

    class LaplacianFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<LaplacianFilter>;

        explicit LaplacianFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  convert_to_uchar_("Convert_to_uchar", true, &parameters_),
                  delta_("Delta", 0, 0, 255, &parameters_),
                  scale_("Scale", 1, 0, 255, &parameters_),
                  size_("Size", 2, 1, 20, &parameters_) {
            setName("LaplacianFilter");
        }

        virtual ~LaplacianFilter() {}

      virtual void apply(cv::Mat &image) {
          if (enable_()) {
              if (image.channels() > 1) {
                  cv::cvtColor(image, image, CV_BGR2GRAY);
              }
              int size = size_() * 2 + 1;

              if (convert_to_uchar_()) {
                  cv::Laplacian(image, image, CV_8U, size, scale_(), delta_(),
                                cv::BORDER_DEFAULT);
              }
        else {
          cv::Laplacian(image, image, CV_32F, size, scale_(), delta_(),
            cv::BORDER_DEFAULT);
        }
      }
    }

  private:
    Parameter<bool> enable_, convert_to_uchar_;
    RangedParameter<double> delta_, scale_;
    RangedParameter<int> size_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SOBEL_H_
