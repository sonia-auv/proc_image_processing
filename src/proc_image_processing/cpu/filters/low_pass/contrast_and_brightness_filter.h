// FACTORY_GENERATOR_CLASS_NAME=ContrastAndBrightnessFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/parallel_loop_body_wrapper.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {
    // Filter showing planes of different analysis (gray, _hsi, _bgr)
    // No threshold
    class ContrastAndBrightnessFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ContrastAndBrightnessFilter>;

        explicit ContrastAndBrightnessFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  contrast_("Contrast", 0, 0, 256, &parameters_,
                            "Contrast"),
                  brightness_("Brightness", 0, -256, 256, &parameters_,
                              "Set Brightness") {
            setName("ContrastAndBrightnessFilter");
        }

        ~ContrastAndBrightnessFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::parallel_for_(cv::Range(0, image.rows * image.cols), ParallelCABF(image, contrast_, brightness_));
        }

    private:
        class ParallelCABF : public ParallelLoopBodyWrapper {
        public:
            explicit ParallelCABF(cv::Mat &image, const RangedParameter<double> &contrast, const RangedParameter<double> &brightness) :
                    image(image),
                    contrast_(contrast),
                    brightness_(brightness) {}

            ~ParallelCABF() override = default;

            void operator()(const cv::Range &range) const override {
                for (auto r = range.start; r < range.end; r++) {
                    int y = r / image.cols;
                    int x = r % image.cols;

                    auto& vec = const_cast<cv::Vec3b &>(image.at<cv::Vec3b>(y, x));
                    for (auto c = 0; c < image.channels(); c++) {
                        vec[c] = cv::saturate_cast<uchar>(contrast_.getValue() * (vec[c]) + brightness_.getValue());
                    }
                }
            }

        private:
            cv::Mat image;
            RangedParameter<double> contrast_, brightness_;
        };

        RangedParameter<double> contrast_, brightness_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
