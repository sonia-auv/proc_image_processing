//
// Created by olavoie on 12/7/17.
//

#ifndef PROC_IMAGE_PROCESSING_WHITE_FILTER_H
#define PROC_IMAGE_PROCESSING_WHITE_FILTER_H


#include <proc_image_processing/filters/filter.h>
#include <math.h>
#include <memory>

namespace proc_image_processing {

    class WhiteFilter : public Filter {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        using Ptr = std::shared_ptr<WhiteFilter>;

        //============================================================================
        // P U B L I C   C / D T O R S

        explicit WhiteFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams), enable_("Enable", false, &parameters_),
                  minimal_1pixel_range_("minimal 1 pixel range", 0, 0, 255, &parameters_),
                  minimal_2pixel_range_("minimal 2 pixel range", 0, 0, 255, &parameters_),
                  minimal_3pixel_range_("minimal 3 pixel range", 0, 0, 255, &parameters_),
                  maximal_1pixel_range_("maximal 1 pixel range", 0, 0, 255, &parameters_),
                  maximal_2pixel_range_("maximal 2 pixel range", 0, 0, 255, &parameters_),
                  maximal_3pixel_range_("maximal 3 pixel range", 0, 0, 255, &parameters_)
        { SetName("WhiteFilter");}

        virtual ~WhiteFilter() {}

        //============================================================================
        // P U B L I C   M E T H O D S
        virtual void Execute(cv::Mat &image){

            cv::Mat mask;

            cv::Scalar min_pixel_range = cv::Scalar(minimal_1pixel_range_(),minimal_2pixel_range_(),minimal_3pixel_range_());
            cv::Scalar max_pixel_range = cv::Scalar(maximal_1pixel_range_(),maximal_2pixel_range_(),maximal_3pixel_range_());

            cv::inRange(image, min_pixel_range, max_pixel_range, mask);

            mask.copyTo(image);

        }




    private:
        //============================================================================
        // P R I V A T E   M E M B E R S
        cv::Mat output_image_;

        Parameter<bool> enable_;
        RangedParameter<int> minimal_1pixel_range_;
        RangedParameter<int> minimal_2pixel_range_;
        RangedParameter<int> minimal_3pixel_range_;
        RangedParameter<int> maximal_1pixel_range_;
        RangedParameter<int> maximal_2pixel_range_;
        RangedParameter<int> maximal_3pixel_range_;

        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_WHITE_FILTER_H
