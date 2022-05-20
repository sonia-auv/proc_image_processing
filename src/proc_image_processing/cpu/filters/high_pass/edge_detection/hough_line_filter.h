// FACTORY_GENERATOR_CLASS_NAME=HoughLineFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_HOUGH_LINE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_HOUGH_LINE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class HoughLineFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<HoughLineFilter>;

        explicit HoughLineFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  rho_("Rho", 1.0f, 0.0f, 1000.0f, &parameters_, "Rho"),
                  theta_("Theta", 1.0f, 0.0f, 1000.0f, &parameters_, "Theta"),
                  min_length_("Minimum length", 1, 0, 1000, &parameters_, "Min length"),
                  max_gap_("Maximum gap", 1, 0, 1000, &parameters_,"Max gap"),
                  threshold_("Threshold", 1, 0, 1000, &parameters_, "Threshold") {
            setName("HoughLineFilter");
        }

        ~HoughLineFilter() override = default;

        void apply(cv::Mat &image) override {
            if (image.channels() > 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }

            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP(
                    image,
                    lines,
                    rho_(),
                    theta_(),
                    threshold_(),
                    min_length_(),
                    max_gap_()
            );

            cv::Mat drawing_image(image.rows, image.cols, CV_8UC3,
                                  cv::Scalar::all(0));
            for (const auto &line : lines) {
                cv::line(
                        drawing_image,
                        cv::Point(line[0], line[1]),
                        cv::Point(line[2], line[3]),
                        cv::Scalar(255, 255, 255),
                        3
                );
            }
            cv::cvtColor(drawing_image, image, CV_BGR2GRAY);
        }

    private:
        RangedParameter<double> rho_, theta_, min_length_, max_gap_;
        RangedParameter<int> threshold_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_HOUGH_LINE_H_
