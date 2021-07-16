/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=RotateFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_ROTATE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_ROTATE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class RotateFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<RotateFilter>;

        explicit RotateFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("enable", false, &parameters_),
                  transpose_("transpose", false, &parameters_),
                  rotate_type_("Rotation_type", 0, 0, 3, &parameters_,
                               "RotateFilter type: 0=NONE, 1=x axis, 2=y axis, 3=all axis") {
            setName("RotateFilter");
        }

        virtual ~RotateFilter() {}

        virtual void apply(cv::Mat &image) {
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

#endif  // PROC_IMAGE_PROCESSING_FILTERS_ROTATE_H_