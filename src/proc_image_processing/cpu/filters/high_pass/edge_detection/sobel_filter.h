/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=SobelFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_SOBEL_H_
#define PROC_IMAGE_PROCESSING_FILTERS_SOBEL_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class SobelFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<SobelFilter>;

        explicit SobelFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  convert_to_uchar_("Convert_to_uchar", true, &parameters_),
                  use_pixel_intensity_correction_("use_pixel_intensity_correction", false,
                                                  &parameters_),
                  delta_("Delta", 0, 0, 255, &parameters_),
                  scale_("Scale", 1, 0, 255, &parameters_),
                  power_pixel_correction_("pixel_correction_power", 1, -10, 10,
                                          &parameters_),
                  size_("Size", 2, 1, 20, &parameters_) {
            setName("SobelFilter");
        }

        virtual ~SobelFilter() {}

        virtual void apply(cv::Mat &image) {
            if (enable_()) {
                if (image.channels() > 1) {
                    cv::cvtColor(image, image, CV_BGR2GRAY);
                }
                cv::Mat sobelX, sobelY;
                int size = size_() * 2 + 1;
                cv::Sobel(image, sobelX, CV_32F, 1, 0, size, scale_(), delta_(),
                          cv::BORDER_DEFAULT);
                cv::Sobel(image, sobelY, CV_32F, 0, 1, size, scale_(), delta_(),
                          cv::BORDER_DEFAULT);

                cv::absdiff(sobelX, 0, sobelX);
                cv::absdiff(sobelY, 0, sobelY);
                cv::addWeighted(sobelX, 0.5, sobelY, 0.5, 0, image, CV_32F);

                if (use_pixel_intensity_correction_()) {
                    for (int y = 0; y < image.rows; y++) {
                        float *ptr = image.ptr<float>(y);
                        for (int x = 0; x < image.cols; x++) {
                            ptr[x] = pow(ptr[x], power_pixel_correction_());
                        }
                    }
                }

                if (convert_to_uchar_()) {
                    cv::convertScaleAbs(image, image);
                }
            }
        }

    private:
        Parameter<bool> enable_, convert_to_uchar_, use_pixel_intensity_correction_;
        RangedParameter<double> delta_, scale_, power_pixel_correction_;
        RangedParameter<int> size_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SOBEL_H_
