// FACTORY_GENERATOR_CLASS_NAME=ContrastAndBrightnessFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class ParallelCABF : public cv::ParallelLoopBody {
        cv::Mat img;
        RangedParameter<double> contrast_, brightness_;

    public:
        ParallelCABF(cv::Mat &img, RangedParameter<double> &contrast, RangedParameter<double> &brightness) :
                img(img),
                contrast_(contrast),
                brightness_(brightness) {}

        void operator()(const cv::Range &range) const override {
            for (auto r = range.start; r < range.end; r++) {
                int y = r / img.cols;
                int x = r % img.cols;

                auto& vec = const_cast<cv::Vec3b &>(img.at<cv::Vec3b>(y, x));
                for (auto c = 0; c < img.channels(); c++) {
                    vec[c] = cv::saturate_cast<uchar>(contrast_.getValue() * (vec[c]) + brightness_.getValue());
                }
            }
        }
    };

    // Filter showing planes of different analysis (gray, _hsi, _bgr)
    // No threshold
    class ContrastAndBrightnessFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ContrastAndBrightnessFilter>;

        explicit ContrastAndBrightnessFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  contrast_("Contrast", 0, 0, 256, &parameters_,
                            "Contrast"),
                  brightness_("Brightness", 0, -256, 256, &parameters_,
                              "Set Brightness") {
            setName("ContrastAndBrightnessFilter");
        }

        ~ContrastAndBrightnessFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                cv::parallel_for_(cv::Range(0, image.rows * image.cols), ParallelCABF(image, contrast_, brightness_));
            }
        }


    private:
        Parameter<bool> enable_;
        RangedParameter<double> contrast_, brightness_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
