// FACTORY_GENERATOR_CLASS_NAME=DeepFilter

#ifndef PROC_IMAGE_PROCESSING_DEEP_H
#define PROC_IMAGE_PROCESSING_DEEP_H

#include "proc_image_processing/cpu/filters/filter.h"
#include "proc_image_processing/color_map.h"
#include <proc_image_processing/cpu/server/target.h>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sonia_common/Detection.h>
#include <sonia_common/DetectionArray.h>

namespace proc_image_processing {
    class DeepFilter : public Filter {

    public:
        explicit DeepFilter(const GlobalParameterHandler &globalParams) :

        Filter(globalParams),
        nh_(ros::NodeHandle("proc_image_processing")),
        debug_contour_("Debug contour", true, &parameters_),
        color_(0, 0, 0) 
        {
            image_subscriber_ = ros::NodeHandle("~").subscribe("/deep_detector/bounding_box", 100, &DeepFilter::callbackBoundingBox, this);
            detectionListing_ = ros::NodeHandle("~").advertiseService("/proc_image_processing/list_deep_color", )
            setName("DeepFilter");

            for(std::pair<std::string, cv::Scalar> pair: COLOR_MAP_DEEP_LEARNING)
            {
                color_keys_.push_back(pair.first);
            }
        };

        ~DeepFilter() override 
        { 
            image_subscriber_.shutdown();
            for(ObjectDesc desc: object_mapping_)
            {
                delete desc.parameter;
            }
        }

        void apply(cv::Mat &image) override 
        {
            Target target;
            image_width_ = image.size().width;
            image_height_ = image.size().height;

            for (sonia_common::Detection &object : bounding_box_) {
                if(object_mapping_.contains(object.class_name.data))
                {
                    if(object_mapping_[object.class_name.data].parameter.getValue())
                    {
                        handleObject(target, object, image, object_mapping_[object.class_name.data].color_scalar);
                    }
                }
                else
                {
                    // if there's still named color
                    if(object_mapping_.size() < color_keys_.size())
                    {
                        //take the next color
                        ObjectDesc desc;
                        desc.color_name = color_keys_[object_param_mapping_.size()];
                        desc.color_scalar = COLOR_MAP_DEEP_LEARNING[color_keys_[object_param_mapping_.size()]];
                        desc.parameter = new Parameter<bool>(object.class_name.data, true, &parameters_);
                        object_mapping_[object.class_name.data] = desc;

                        handleObject(target, object, image, object_mapping_[object.class_name.data].color_scalar);
                    }

                    // generate random color if there's isn't
                    else
                    {
                        srand(time(NULL));
                        ObjectDesc desc;
                        desc.color_name = "random";
                        desc.color_scalar = cv::Scalar(rand()%255, rand()%255, rand()%255);
                        desc.parameter = new Parameter<bool>(object.class_name.data, true, &parameters_);
                        object_mapping_[object.class_name.data] = desc;

                        handleObject(target, object, image, object_mapping_[object.class_name.data].color_scalar);
                    }
                }
            }

            for (int i = 0; i < (int) objects_.size(); ++i) {
                notify(objects_.back());
                objects_.pop_back();
            }
        };

    private:

        typedef struct
        {
            std::string color_name;
            cv::Scalar color_scalar;
            Parameter<bool> parameter;
        } ObjectDesc;

        typedef struct
        {
            int center_x;
            int center_y;

            int size_x;
            int size_y;
        } ObjectBoundingBox;

        const int BBOX_INFO_RECT_HEIGHT = 30;
        const int BBOX_X_TOP_LEFT_CORRECTION = 5;
        const int BBOX_X_BOTTOM_RIGHT_CORRECTION = 10;
        const cv::Scalar BBOX_INFO_TEXT_COLOR = cv::Scalar(255, 255, 255);
        const int BBOX_INFO_FONT = cv::FONT_HERSHEY_TRIPLEX;

        // TODO why do we need this here? Every other filters doesn't need a NodeHandle/Subscriber
        ros::Subscriber image_subscriber_;
        ros::ServiceServer detectionListing_;
        ros::NodeHandle nh_;

        std::map<std::string, ObjectDesc> object_mapping_;
        std::vector<std::string> color_keys_;
        std::vector<sonia_common::Detection> bounding_box_;
        std::vector<Target> objects_;
        Parameter<bool> debug_contour_;
        int image_width_{};
        int image_height_{};

        void callbackBoundingBox(const sonia_common::DetectionArrayConstPtr &msg) 
        {
            if (bounding_box_.empty()) 
            {
                bounding_box_.clear();
            }
            bounding_box_ = msg->detected_object;
        }

        bool DetectionListing(sonia_common::ListDeepColor::Request &req, sonia_common::ListDeepColor::Response &res)
        {
            std::string list = "";
            for(std::pair<std::string, ObjectDesc> pair: object_mapping_)
            {
                list += pair.first + "," + pair.second.color_name + ",";
            }

            // delete trailling ,
            list.pop_back();

            res.list = list;
        }

        static inline ObjectBoundingBox normalizeBoundingBox(const sonia_common::Detection &object)
        {
            ObjectBoundingBox box;
            box.size_x = (object.right - object.left)*image_width_;
            box.size_y = (object.bottom - object.top)*image_height_;
            box.center_x = (object.left*image_width_)+(box.size_x/2);
            box.center_y = (object.top*image_height_)+(box.size_y/2);
            return box;
        }

        static inline std::string convertFloatToString(float value) 
        {
            value = value * 100;
            std::ostringstream ss;
            ss << std::setprecision(4);
            ss << value;
            return ss.str();
        }

        inline void buildTarget(Target &target, const sonia_common::Detection &object, const ObjectBoundingBox box) const 
        {
            int image_central_x;
            int image_central_y;
            int bounding_box_center_x;
            int bounding_box_center_y;
            int vision_bounding_box_center_x;
            int vision_bounding_box_center_y;

            image_central_x = image_width_ / 2;
            image_central_y = image_height_ / 2;

            vision_bounding_box_center_x = box.center_x - image_central_x;
            vision_bounding_box_center_y = image_central_y - box.center_y;

            target.setCenter(vision_bounding_box_center_x, vision_bounding_box_center_y);
            target.setSize(box.size_x, box.size_y);
            target.setSpecialField1(object.class_name.data);
            target.setSpecialField2(convertFloatToString(object.confidence));
        }

        inline void drawTarget(
                cv::Mat &image,
                const sonia_common::Detection &object,
                const ObjectBoundingBox box,
                int thickness = 3,
                const cv::Scalar &color_box = cv::Scalar(0, 255, 0))
        {
            int origin_x = (box.center_x - (int) (box.size_x / 2));
            int origin_y = (box.center_y - (int) (box.size_y / 2));

            int top_left_x = origin_x - BBOX_X_TOP_LEFT_CORRECTION;
            int top_left_y = origin_y - BBOX_INFO_RECT_HEIGHT;
            int bottom_right_x = box.size_x + BBOX_X_BOTTOM_RIGHT_CORRECTION;
            int bottom_right_y = BBOX_INFO_RECT_HEIGHT;

            cv::Rect rect_top(top_left_x, top_left_y, bottom_right_x, bottom_right_y);
            cv::Rect rect(origin_x, origin_y, box.size_x, box.size_y);

            cv::rectangle(image, rect, color_box, thickness);
            cv::rectangle(image, rect_top, color_box, CV_FILLED);

            std::string text = createTextBoundingBox(object);
            cv::putText(image, text, cv::Point(origin_x, origin_y), BBOX_INFO_FONT, 1, BBOX_INFO_TEXT_COLOR, 2, CV_AA);
        }

        static inline std::string createTextBoundingBox(const sonia_common::Detection &object) 
        {
            std::stringstream ss;
            ss << object.class_name.data << ":" << convertFloatToString(object.confidence) << "%";
            return ss.str();
        }

        inline void
        handleObject(Target &target, const sonia_common::Detection &object, cv::Mat &image, const cv::Scalar &color) 
        {
            ObjectBoundingBox box = normalizeBoundingBox(object);
            buildTarget(target, box);
            if (debug_contour_()) 
            {
                drawTarget(image, box, 10, color);
            }
            objects_.push_back(target);
        }
    };

} //proc_image_processing

#endif //PROC_IMAGE_PROCESSING_DEEP_2019_H

