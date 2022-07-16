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
                  draw_on_original_("Draw on original image", false, &parameters_),
                  rho_("Rho", 1.0f, 0.0f, 1000.0f, &parameters_, "Rho"),
                  theta_("Theta - angle", 1, 0, 360, &parameters_, "Theta - angle"),
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
                    theta_() * CV_PI/180, 
                    threshold_(),
                    min_length_(),
                    max_gap_()
            );


            
            cv::Mat drawing_image(image.rows, image.cols, CV_8UC3,
                                cv::Scalar::all(0));

            if(draw_on_original_()){ // Draw on original image
                drawing_image = global_param_handler_.getOriginalImage();
            }

            for (const auto &line : lines) {
                cv::line(
                        drawing_image,
                        cv::Point(line[0], line[1]),
                        cv::Point(line[2], line[3]),
                        cv::Scalar(0, 100, 255),
                        2
                );
            }

            if(draw_on_original_()){
                image = drawing_image;
            }else{
                cv::cvtColor(drawing_image, image, CV_BGR2GRAY);
            }

        }

    private:
        //Custom order of variable
        Parameter<bool> draw_on_original_;
        RangedParameter<double>  min_length_, max_gap_;
        RangedParameter<int> threshold_, theta_;
        RangedParameter<double> rho_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_HOUGH_LINE_H_
