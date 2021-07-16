/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=HoughLineFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_HOUGH_LINE_H_
#define PROC_IMAGE_PROCESSING_FILTERS_HOUGH_LINE_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class HoughLineFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<HoughLineFilter>;

        explicit HoughLineFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  rho_("Rho", 1.0f, 0.0f, 1000.0f, &parameters_),
                  theta_("Theta", 1.0f, 0.0f, 1000.0f, &parameters_),
                  min_length_("Min_length", 1, 0, 1000, &parameters_),
                  max_gap_("Max_gap", 1, 0, 1000, &parameters_),
                  threshold_("ThresholdFilter", 1, 0, 1000, &parameters_) {
            setName("HoughLineFilter");
        }

        virtual ~HoughLineFilter() {}

        virtual void apply(cv::Mat &image) {
            if (enable_()) {
                if (image.channels() > 1) {
                    cv::cvtColor(image, image, CV_BGR2GRAY);
                }

                std::vector<cv::Vec4i> lines;
                cv::HoughLinesP(image, lines, rho_(), theta_(), threshold_(),
                                min_length_(), max_gap_());

                cv::Mat drawing_image(image.rows, image.cols, CV_8UC3,
                                      cv::Scalar::all(0));
                for (const auto &line : lines) {
                    cv::line(drawing_image, cv::Point(line[0], line[1]),
                             cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 3);
                }
                cv::cvtColor(drawing_image, image, CV_BGR2GRAY);
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<double> rho_, theta_, min_length_, max_gap_;
        RangedParameter<int> threshold_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SOBEL_H_
