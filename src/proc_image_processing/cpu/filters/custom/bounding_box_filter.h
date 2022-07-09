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
                heigth_("Height", 0, 0, 600, &parameters_, "Heigth"),
                origin_x_("Origin X", 0, -300, 300, &parameters_, "Origin X"),
                origin_y_("Origin Y", 0, -300, 300, &parameters_, "Origin Y"),
                thickness_("Thickness", 1, 1, 10, &parameters_, "Thickness") {
            setName("BoundingBoxFilter");
        }

        ~BoundingBoxFilter() override = default;

        void apply(cv::Mat &image) override {
            cv::Rect rect((image.cols/2)+origin_x_()-width_()/2, (image.rows/2)-origin_y_()-heigth_()/2, width_(), heigth_());

            cv::rectangle(image, rect, cv::Scalar(0,255,0), thickness_());
        }

    private:
        RangedParameter<int> width_, heigth_, thickness_, origin_x_, origin_y_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_
