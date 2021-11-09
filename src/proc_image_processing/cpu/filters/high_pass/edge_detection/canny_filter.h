// FACTORY_GENERATOR_CLASS_NAME=CannyFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_CANNY_H_
#define PROC_IMAGE_PROCESSING_FILTERS_CANNY_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class CannyFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<CannyFilter>;

        explicit CannyFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  l2_gradiant_("L2 gradient", false, &parameters_),
                  thresh_one_("Threshold 1", 100, 0, 255, &parameters_),
                  thresh_two_("Threshold 2", 200, 0, 255, &parameters_),
                  aperture_size_("Aperture size", 3, 0, 20, &parameters_) {
            setName("CannyFilter");
        }

        ~CannyFilter() override = default;

        void apply(cv::Mat &image) override {
            if (image.channels() > 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }
            cv::Canny(image, image, thresh_one_(), thresh_two_(),
                      aperture_size_() * 2 + 1, l2_gradiant_());
        }

    private:
        Parameter<bool> l2_gradiant_;
        RangedParameter<int> thresh_one_;
        RangedParameter<int> thresh_two_;
        RangedParameter<int> aperture_size_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_CANNY_H_
