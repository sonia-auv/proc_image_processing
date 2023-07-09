// FACTORY_GENERATOR_CLASS_NAME=BoundingBoxFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_
#define PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>
#include <sonia_common/CenterShapeBoundingBox.h>

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
                thickness_("Thickness", 1, 1, 10, &parameters_, "Thickness"),
                center_width_("Center Width", 0, 0, 600, &parameters_, "Width"),
                center_heigth_("Center Height", 0, 0, 600, &parameters_, "Heigth"),
                center_thickness_("Center Thickness", 1, 1, 10, &parameters_, "Thickness") {
            setName("BoundingBoxFilter");
            m_box_pub = this->nh_->advertise<sonia_common::CenterShapeBoundingBox>("/proc_image_processing/gate_box", 100);
            ROS_INFO("BoundingBox Filter Created Filter Created");
        }

        ~BoundingBoxFilter() override = default;

        void apply(cv::Mat &image) override {
            ROS_INFO("Image aquired");
            cv::Rect rect((image.cols/2)+origin_x_()-width_()/2, (image.rows/2)-origin_y_()-heigth_()/2, width_(), heigth_());
            cv::Rect center_rect((image.cols/2)+origin_x_()-center_width_()/2, (image.rows/2)-origin_y_()-center_heigth_()/2, center_width_(), center_heigth_());

            cv::rectangle(image, rect, cv::Scalar(0,255,0), thickness_());
            cv::rectangle(image, center_rect, cv::Scalar(255,0,0), center_thickness_());

            sonia_common::CenterShapeBoundingBox box;
            sonia_common::BoundingBox2D cntr;
            sonia_common::BoundingBox2D objt;
            geometry_msgs::Pose2D pose;

            pose.x = image.cols/2;
            pose.y = image.rows/2;
            objt.center = pose;
            objt.size_x = width_();
            objt.size_y = heigth_();

            cntr.center = pose;
            objt.size_x = center_width_();
            objt.size_y = center_heigth_();

            box.center = cntr;
            box.shape = objt;

            m_box_pub.publish(box);
            ROS_INFO("MSG Published");
        }

    private:
        RangedParameter<int> width_;
        RangedParameter<int> heigth_;
        RangedParameter<int> origin_x_;
        RangedParameter<int> origin_y_;
        RangedParameter<int> thickness_;

        RangedParameter<int> center_width_;
        RangedParameter<int> center_heigth_;
        RangedParameter<int> center_thickness_;

        ros::Publisher m_box_pub;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_BOUNDING_BOX_H_
