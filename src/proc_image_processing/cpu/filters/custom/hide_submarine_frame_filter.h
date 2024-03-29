// FACTORY_GENERATOR_CLASS_NAME=HideSubmarineFrameFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_SUBMARINE_FRAME_MASKER_H_
#define PROC_IMAGE_PROCESSING_FILTERS_SUBMARINE_FRAME_MASKER_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class HideSubmarineFrameFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<HideSubmarineFrameFilter>;

        explicit HideSubmarineFrameFilter(const GlobalParameterHandler &globalParams) :
                Filter(globalParams),
                rotate_type_(
                        "Rotation type",
                        0,
                        0,
                        3,
                        &parameters_,
                        std::string("RotateFilter type: 0=NONE, 1=x axis, 2=y axis, 3=all axis")
                ) {
            setName("HideSubmarineFrameFilter");
            // TODO Fix this hardcoded file that isn't part of the project
            //std::string mask_name = std::string(getenv("SONIA_WORKSPACE_ROOT")) +
            //                        std::string("/ros/src/vision_server/config/bottom_mask.jpg");
            //bottom_mask_ = cv::imread(mask_name, CV_LOAD_IMAGE_GRAYSCALE);
        }

        ~HideSubmarineFrameFilter() override = default;

        void apply(cv::Mat &image) override {
            if (prev_rot_value_ != rotate_type_()) {
                prev_rot_value_ = rotate_type_();
                switch (rotate_type_()) {
                    case 1:
                        cv::flip(bottom_mask_, bottom_mask_, 0);
                        break;
                    case 2:
                        cv::flip(bottom_mask_, bottom_mask_, 1);
                        break;
                    case 3:
                        cv::flip(bottom_mask_, bottom_mask_, -1);
                        break;
                    default:
                        break;
                }
            }
            if (image.size() == bottom_mask_.size())
                cv::bitwise_and(image, bottom_mask_, image);
        }

    private:
        RangedParameter<int> rotate_type_;
        cv::Mat bottom_mask_;
        int prev_rot_value_ = 0;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SUBMARINE_FRAME_MASKER_H_
