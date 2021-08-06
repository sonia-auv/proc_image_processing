// FACTORY_GENERATOR_CLASS_NAME=ScharrAddingFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_SCHARR_ADDING_H_
#define PROC_IMAGE_PROCESSING_FILTERS_SCHARR_ADDING_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class ScharrAddingFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ScharrAddingFilter>;

        explicit ScharrAddingFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  run_small_image_("Run small image", true, &parameters_,
                                   "Resize image to run on smaller image"),
                  convert_to_uchar_("Convert to uchar", false, &parameters_),
                  delta_("Delta", 0, 0, 255, &parameters_),
                  scale_("Scale", 1, 0, 255, &parameters_),
                  mean_factor_("Mean factor", 1.0f, 0.0f, 10.0f, &parameters_),
                  plane_blue_("Blue", false, &parameters_),
                  plane_green_("Green", false, &parameters_),
                  plane_red_("Red", false, &parameters_),
                  plane_hue_("Hue", false, &parameters_),
                  plane_saturation_("Saturation", false, &parameters_),
                  plane_intensity_("Intensity", false, &parameters_),
                  plane_gray_("Gray", false, &parameters_) {
            setName("ScharrAddingFilter");
        }

        ~ScharrAddingFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                if (image.channels() != 3) return;
                if (run_small_image_()) {
                    cv::resize(image, image, cv::Size(image.cols / 2, image.rows / 2));
                }

                std::vector<cv::Mat> colorPlanes = getColorPlanes(image);
                cv::Mat sum = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);

                if (plane_blue_()) cv::add(getScharr(colorPlanes[0]), sum, sum);
                if (plane_green_()) cv::add(getScharr(colorPlanes[1]), sum, sum);
                if (plane_red_()) cv::add(getScharr(colorPlanes[2]), sum, sum);
                if (plane_hue_()) cv::add(getScharr(colorPlanes[3]), sum, sum);
                if (plane_saturation_()) cv::add(getScharr(colorPlanes[4]), sum, sum);
                if (plane_intensity_()) cv::add(getScharr(colorPlanes[5]), sum, sum);
                if (plane_gray_()) cv::add(getScharr(colorPlanes[6]), sum, sum);

                sum.copyTo(image);
                if (run_small_image_()) {
                    cv::resize(image, image, cv::Size(image.cols * 2, image.rows * 2));
                }

                if (convert_to_uchar_() && image.channels() < 3) {
                    cv::cvtColor(image, image, CV_GRAY2BGR);
                }
            }
        }

    private:
        cv::Mat getScharr(const cv::Mat &img) {
            cv::Mat abs_x, scharrX, abs_y, scharrY, diff;

            cv::Scharr(img, scharrX, CV_32F, 1, 0, scale_(), delta_(),
                       cv::BORDER_REPLICATE);
            cv::Scharr(img, scharrY, CV_32F, 0, 1, scale_(), delta_(),
                       cv::BORDER_REPLICATE);
            cv::absdiff(scharrX, 0, scharrX);
            cv::absdiff(scharrY, 0, scharrY);

            cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, diff, CV_32F);

            cv::Scalar mean = cv::mean(diff);
            cv::threshold(diff, diff, (mean[0] * mean_factor_()), 0,
                          CV_THRESH_TOZERO);

            return diff;
        }

        // _run_small_image accelerate the pipeline by
        // reducing the image size by two (in each direction)
        // so that the scharr computation does not take to much time
        // when multiple images.
        Parameter<bool> enable_, run_small_image_, convert_to_uchar_;
        // _mean_multiplier act as threshold for noise.
        // When set, it remove everything under the mean to keep only
        // proeminent contours.
        RangedParameter<double> delta_, scale_, mean_factor_;
        Parameter<bool> plane_blue_, plane_green_, plane_red_;
        Parameter<bool> plane_hue_, plane_saturation_, plane_intensity_;
        Parameter<bool> plane_gray_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SCHARR_ADDING_H_
