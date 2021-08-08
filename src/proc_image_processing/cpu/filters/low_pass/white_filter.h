// FACTORY_GENERATOR_CLASS_NAME=WhiteFilter

#ifndef PROC_IMAGE_PROCESSING_WHITE_FILTER_H
#define PROC_IMAGE_PROCESSING_WHITE_FILTER_H


#include "proc_image_processing/cpu/filters/filter.h"
#include <cmath>
#include <memory>

namespace proc_image_processing {

    class WhiteFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<WhiteFilter>;

        explicit WhiteFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  minimal_1pixel_range_("Minimal 1 pixel range", 0, 0, 255, &parameters_),
                  minimal_2pixel_range_("Minimal 2 pixel range", 0, 0, 255, &parameters_),
                  minimal_3pixel_range_("Minimal 3 pixel range", 0, 0, 255, &parameters_),
                  maximal_1pixel_range_("Maximal 1 pixel range", 0, 0, 255, &parameters_),
                  maximal_2pixel_range_("Maximal 2 pixel range", 0, 0, 255, &parameters_),
                  maximal_3pixel_range_("Maximal 3 pixel range", 0, 0, 255, &parameters_) {
            setName("WhiteFilter");
        }

        ~WhiteFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::Mat mask;

            cv::Scalar min_pixel_range = cv::Scalar(
                    minimal_1pixel_range_(),
                    minimal_2pixel_range_(),
                    minimal_3pixel_range_()
            );
            cv::Scalar max_pixel_range = cv::Scalar(
                    maximal_1pixel_range_(),
                    maximal_2pixel_range_(),
                    maximal_3pixel_range_()
            );

            cv::inRange(image, min_pixel_range, max_pixel_range, image);
        }

    private:
        cv::Mat output_image_;

        RangedParameter<int> minimal_1pixel_range_;
        RangedParameter<int> minimal_2pixel_range_;
        RangedParameter<int> minimal_3pixel_range_;
        RangedParameter<int> maximal_1pixel_range_;
        RangedParameter<int> maximal_2pixel_range_;
        RangedParameter<int> maximal_3pixel_range_;
    };

}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_WHITE_FILTER_H
