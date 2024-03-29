// FACTORY_GENERATOR_CLASS_NAME=ScharrFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_SCHARR_H_
#define PROC_IMAGE_PROCESSING_FILTERS_SCHARR_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class ScharrFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ScharrFilter>;

        explicit ScharrFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  convert_to_uchar_("Convert to uchar", true, &parameters_),
                  use_pixel_intensity_correction_("Pixel intensity correction", false, &parameters_),
                  delta_("Delta", 0, 0, 255, &parameters_),
                  scale_("Scale", 1, 0, 255, &parameters_),
                  power_pixel_correction_("Pixel correction power", 1, -10, 10, &parameters_) {
            setName("ScharrFilter");
        }

        ~ScharrFilter() override = default;

        void apply(cv::Mat &image) override {
            if (image.channels() > 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }
            cv::Mat scharrX, scharrY;
            cv::Scharr(image, scharrX, CV_32F, 1, 0, scale_(), delta_(),
                       cv::BORDER_REPLICATE);
            cv::Scharr(image, scharrY, CV_32F, 0, 1, scale_(), delta_(),
                       cv::BORDER_REPLICATE);
            cv::absdiff(scharrX, 0, scharrX);
            cv::absdiff(scharrY, 0, scharrY);

            cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, image, CV_32F);

            if (use_pixel_intensity_correction_()) {
                for (int y = 0; y < image.rows; y++) {
                    auto *ptr = image.ptr<float>(y);
                    for (int x = 0; x < image.cols; x++) {
                        ptr[x] = pow(ptr[x], power_pixel_correction_());
                    }
                }
            }

            if (convert_to_uchar_()) {
                cv::convertScaleAbs(image, image);
            }
        }

    private:
        Parameter<bool> convert_to_uchar_;
        Parameter<bool> use_pixel_intensity_correction_;
        RangedParameter<double> delta_;
        RangedParameter<double> scale_;
        RangedParameter<double> power_pixel_correction_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SCHARR_H_
