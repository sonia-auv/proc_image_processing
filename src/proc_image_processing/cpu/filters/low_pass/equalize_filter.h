/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=EqualizeFilter

#ifndef PROVIDER_VISION_FILTERS_EQUALIZE_H_
#define PROVIDER_VISION_FILTERS_EQUALIZE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class EqualizeFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<EqualizeFilter>;

        explicit EqualizeFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("enable", false, &parameters_) {
            setName("EqualizeFilter");
        }

        virtual ~EqualizeFilter() {}

      virtual void apply(cv::Mat &image) {
          if (enable_()) {
              cv::equalizeHist(image, image);
          }
      }

  private:
    Parameter<bool> enable_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_EQUALIZE_H_
