// FACTORY_GENERATOR_CLASS_NAME=BoundingBoxFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_
#define PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class BoundingBoxFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<BoundingBoxFilter>;

        explicit BoundingBoxFilter(const GlobalParameterHandler &globalParams) :
                Filter(globalParams),
                width_("Width", 0, 0, 600, &parameters_, "Width"),
                heigth_("Height", 0, 0, 400, &parameters_, "Heigth"),
                origin_x_("Origin X", 0, -600, 600, &parameters_, "Origin X"),
                origin_y_("Origin Y", 0, -400, 400, &parameters_, "Origin Y"),
                thickness_("Thickness", 1, 1, 10, &parameters_, "Thickness") {
            setName("BoundingBoxFilter");
        }

        ~BoundingBoxFilter() override = default;

        void apply(cv::Mat &image) override {
            //image.copyTo(output_image_);

            cv::Rect rect(origin_x_(), origin_y_(), width_(), heigth_());

            cv::rectangle(image, rect, cv::Scalar(0,255,0), thickness_());

        }

    private:
        RangedParameter<int> width_, heigth_, thickness_, origin_x_, origin_y_;

        //cv::Mat output_image_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_
