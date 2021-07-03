/// \author	Antoine Dozois <dozois.a@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_DEEP_2019_H
#define PROC_IMAGE_PROCESSING_DEEP_2019_H

#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sonia_common/Detection.h>
#include <sonia_common/DetectionArray.h>

namespace proc_image_processing {
    class Deep2019 : public Filter {

    public:
        explicit Deep2019(const GlobalParamHandler& globalParams) :
            Filter(globalParams),
            nh_(ros::NodeHandle("proc_image_processing")),
            debug_contour_("Debug contour", false, &parameters_),
            vetalas_("vetalas", true, &parameters_),
            draugr_("draugr", true, &parameters_),
            jiangshi_("jiangshi", true, &parameters_),
            answag_("answag", true, &parameters_),
            vampire_("vampire", true, &parameters_),
            bat_("bat", true, &parameters_),
            wolf_("wolf", true, &parameters_),
            color_(0, 0, 0) {
            image_subscriber_ = ros::NodeHandle("~").subscribe("/deep_detector/bounding_box", 100, &Deep2019::boundingBoxCallback, this);
            SetName("Deep2019");
        };

        virtual ~Deep2019() { image_subscriber_.shutdown(); }

        virtual void ApplyFilter(cv::Mat& image) {
                Target target;
                image_width_ = image.size().width;
                image_height_ = image.size().height;

                for (auto& object : bounding_box_) {
                    if (vetalas_.GetValue() && object.class_name.data == vetalas_.GetName()) {
                        color_ = cv::Scalar(0, 0, 255);
                        handleObject(target, object, image, color_);
                    }
                    if (draugr_.GetValue() && object.class_name.data == draugr_.GetName()) {
                        color_ = cv::Scalar(0, 255, 0);
                        handleObject(target, object, image, color_);
                    }
                    if (jiangshi_.GetValue() && object.class_name.data == jiangshi_.GetName()) {
                        color_ = cv::Scalar(255, 0, 0);
                        handleObject(target, object, image, color_);
                    }
                    if (answag_.GetValue() && object.class_name.data == answag_.GetName()) {
                        color_ = cv::Scalar(244, 185, 66);
                        handleObject(target, object, image, color_);
                    }
                    if (vampire_.GetValue() && object.class_name.data == vampire_.GetName()) {
                        color_ = cv::Scalar(200, 185, 66);
                        handleObject(target, object, image, color_);
                    }
                    if (bat_.GetValue() && object.class_name.data == bat_.GetName()) {
                        color_ = cv::Scalar(217, 244, 66);
                        handleObject(target, object, image, color_);
                    }
                    if (wolf_.GetValue() && object.class_name.data == wolf_.GetName()) {
                        color_ = cv::Scalar(66, 244, 223);
                        handleObject(target, object, image, color_);
                    }
                }

                for (int i = 0; i < (int)objects_.size(); ++i) {
                    NotifyTarget(objects_.back());
                    objects_.pop_back();
                }
        };

    private:
        const int BBOX_INFO_RECT_HEIGHT = 30;
        const int BBOX_X_TOP_LEFT_CORRECTION = 5;
        const int BBOX_X_BOTTOM_RIGHT_CORRECTION = 10;
        const cv::Scalar BBOX_INFO_TEXT_COLOR = cv::Scalar(255, 255, 255);
        const int BBOX_INFO_FONT = cv::FONT_HERSHEY_TRIPLEX;

        ros::Subscriber image_subscriber_;
        ros::NodeHandle nh_;
        std::vector<sonia_common::Detection> bounding_box_;
        std::vector<Target> objects_;
        Parameter<bool> debug_contour_, vetalas_, draugr_, jiangshi_, answag_, vampire_, bat_, wolf_;
        int image_width_;
        int image_height_;
        cv::Scalar color_;

        void boundingBoxCallback(const sonia_common::DetectionArrayConstPtr& msg) {
            if (bounding_box_.empty())
                bounding_box_.clear();
            bounding_box_ = msg->detected_object;
        }

        inline std::string convertFloatToString(float value) {
            value = value * 100;
            std::ostringstream ss;
            ss << std::setprecision(4);
            ss << value;
            return ss.str();
        }

        inline void constructTarget(Target& target, const sonia_common::Detection& object) {
            int image_central_x;
            int image_central_y;
            int bounding_box_center_x;
            int bounding_box_center_y;
            int vision_bounding_box_center_x;
            int vision_bounding_box_center_y;

            image_central_x = (int)(image_width_ / 2);
            image_central_y = (int)(image_height_ / 2);

            bounding_box_center_x = (int)object.bbox.center.x;
            bounding_box_center_y = (int)object.bbox.center.y;

            vision_bounding_box_center_x = bounding_box_center_x - image_central_x;
            vision_bounding_box_center_y = image_central_y - bounding_box_center_y;

            target.SetCenter(vision_bounding_box_center_x, vision_bounding_box_center_y);
            target.SetSize((int)object.bbox.size_x, (int)object.bbox.size_y);
            target.SetSpecField_1(object.class_name.data);
            target.SetSpecField_2(convertFloatToString(object.confidence));
        }

        inline void drawTarget(cv::Mat& image, const sonia_common::Detection& object, int thickness = 3, const cv::Scalar& color_box = cv::Scalar(0, 255, 0)) {
            int origin_x = (int)(object.bbox.center.x - (object.bbox.size_x / 2));
            int origin_y = (int)(object.bbox.center.y - (int)(object.bbox.size_y / 2));

            int top_left_x = origin_x - BBOX_X_TOP_LEFT_CORRECTION;
            int top_left_y = origin_y - BBOX_INFO_RECT_HEIGHT;
            int bottom_right_x = (int)object.bbox.size_x + BBOX_X_BOTTOM_RIGHT_CORRECTION;
            int bottom_right_y = BBOX_INFO_RECT_HEIGHT;

            cv::Rect rect_top(top_left_x, top_left_y, bottom_right_x, bottom_right_y);
            cv::Rect rect(origin_x, origin_y, (int)object.bbox.size_x, (int)object.bbox.size_y);

            cv::rectangle(image, rect, color_box, thickness);
            cv::rectangle(image, rect_top, color_box, CV_FILLED);

            std::string text = creatTextBoundingBox(object);
            cv::putText(image, text, cv::Point(origin_x, origin_y), BBOX_INFO_FONT, 1, BBOX_INFO_TEXT_COLOR, 2, CV_AA);
        }

        inline std::string creatTextBoundingBox(const sonia_common::Detection& object) {
            std::stringstream ss;
            ss << object.class_name.data << ":" << convertFloatToString(object.confidence) << "%";
            return ss.str();
        }

        inline void handleObject(Target& target, const sonia_common::Detection& object, cv::Mat& image, const cv::Scalar& color) {
            constructTarget(target, object);
            if (debug_contour_()) {
                drawTarget(image, object, 10, color);
            }
            objects_.push_back(target);
        }
    };

} //proc_image_processing

#endif //PROC_IMAGE_PROCESSING_DEEP_DICE_H
;

