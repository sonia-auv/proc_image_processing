/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=ContrastAndBrightnessFilter

#ifndef PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include "filter.h"
#include <memory>

namespace proc_image_processing {

    // Filter showing planes of different analysis (gray, _hsi, _bgr)
    // No threshold
    class ContrastAndBrightnessFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ContrastAndBrightnessFilter>;

        explicit ContrastAndBrightnessFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("enable", false, &parameters_),
                  contrast_("Contrast", 0, 0, 256, &parameters_,
                            "Contrast"),
                  brightness_("Brightness", 0, -256, 256, &parameters_,
                              "Set Brightness"),
                  rows_(0),
                  cols_(0) {
            setName("ContrastAndBrightnessFilter");
        }

        virtual ~ContrastAndBrightnessFilter() {}

        virtual void apply(cv::Mat &image) {
            if (enable_()) {
                rows_ = image.rows;
                cols_ = image.cols;

                // Set result matrices
                cv::Mat result = cv::Mat::zeros(rows_, cols_, image.type());

                // Replace with new images
                for (int y = 0; y < image.rows; y++) {
                    for (int x = 0; x < image.cols; x++) {
                        for (int c = 0; c < image.channels(); c++)
                            result.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                                    contrast_() * (image.at<cv::Vec3b>(y, x)[c]) + brightness_());
                    }
                }
                result.copyTo(image);
            }
        }


    private:
        Parameter<bool> enable_;
        RangedParameter<double> contrast_, brightness_;
        // Color matrices
        int rows_;
        int cols_;
    };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_
