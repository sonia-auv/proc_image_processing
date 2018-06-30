/// \author	Antoine Dozois <dozois.a@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#ifndef PROC_IMAGE_PROCESSING_DEEP_DICE_H
#define PROC_IMAGE_PROCESSING_DEEP_DICE_H

#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "proc_image_processing/Detection.h"
#include "proc_image_processing/DetectionArray.h"



namespace proc_image_processing {
class DeepDice : public Filter{

public:

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit DeepDice(const GlobalParamHandler &globalParams):
            Filter(globalParams),
            nh_(ros::NodeHandle("proc_image_processing")),
            enable_("Enable", false, &parameters_),
            debug_contour_("Debug contour", false, &parameters_),
            dice1_("dice1", true, &parameters_),
            dice2_("dice2", true, &parameters_),
            dice5_("dice5", true, &parameters_),
            dice6_("dice6", true, &parameters_)
    {
        image_subscriber_= ros::NodeHandle("~").subscribe("/deep_detector/bounding_box", 100, &DeepDice::boundingBoxCallback, this);
        SetName("DeepDice");
    };

    virtual ~DeepDice() {}

    //============================================================================
    // P U B L I C   M E T H O D S
    //----------------------------------------------------------------------------
    //

    virtual void Execute(cv::Mat &image){
        if(enable_()){
            Target target;
            image_width_ = image.size().width;
            image_height_ = image.size().height;
            for (auto &object: bounding_box_) {
                if(dice1_.GetValue() && object.class_name.data == dice1_.GetName())  {
                    handleObject(target, object, image);
                }
                if(dice2_.GetValue() && object.class_name.data == dice2_.GetName())  {
                    handleObject(target, object, image);
                }
                if(dice5_.GetValue() && object.class_name.data == dice5_.GetName())  {
                    handleObject(target, object, image);
                }
                if(dice6_.GetValue() && object.class_name.data == dice6_.GetName())  {
                    handleObject(target, object, image);
                }
            }

            for (int i = 0; i < (int)objects_.size(); ++i) {
                NotifyTarget(objects_.back());
                objects_.pop_back();
            }
        }
    };


private:
    ros::Subscriber image_subscriber_;
    ros::NodeHandle nh_;
    std::vector<Detection> bounding_box_;
    std::vector<Target> objects_;
    Parameter<bool> enable_, debug_contour_, dice1_, dice2_, dice5_, dice6_;
    int image_width_;
    int image_height_;

    void boundingBoxCallback(const DetectionArrayConstPtr &msg){
        if (bounding_box_.empty())
            bounding_box_.clear();
        bounding_box_ = msg->detected_object;

    }

    inline std::string convertFloatToString(float value){
        value = value * 100;
        std::ostringstream ss;
        ss << std::setprecision(4);
        ss << value;
        return ss.str();
    }

    inline void constructTarget(Target &target, const Detection &object){
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

        ROS_INFO_STREAM("image_central_x:" << image_central_x);
        ROS_INFO_STREAM("image_central_y:" << image_central_y);
        ROS_INFO_STREAM("bbox_central_x:" << bounding_box_center_x);
        ROS_INFO_STREAM("bbox_central_y:" << bounding_box_center_y);
        ROS_INFO_STREAM("Final x:" << vision_bounding_box_center_x);
        ROS_INFO_STREAM("Final y:" << vision_bounding_box_center_y);
        
        target.SetCenter(vision_bounding_box_center_x, vision_bounding_box_center_y);
        //target.SetCenter((int)object.bbox.center.x, (int)object.bbox.center.y);
        target.SetSize((int)object.bbox.size_x, (int)object.bbox.size_y);
        target.SetSpecField_1(object.class_name.data);
        target.SetSpecField_2(convertFloatToString(object.confidence));
    }

    inline void drawTarget(cv::Mat &image, const Detection &object, int thickness=3, const cv::Scalar &color_box=cv::Scalar(0, 255, 0)){
        auto origin_x = (int)(object.bbox.center.x - (object.bbox.size_x/2));
        auto origin_y = (int)(object.bbox.center.y - (int)(object.bbox.size_y/2));
        cv::Rect rect(origin_x, origin_y, (int)object.bbox.size_x, (int)object.bbox.size_y);
        cv::rectangle(image, rect, color_box, thickness);
        std::string text = creatTextBoundingBox(object);
        cv::putText(image, text, cv::Point(origin_x,origin_y - 20), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,255), 2, CV_AA);
    }

    inline std::string creatTextBoundingBox(const Detection &object){
        std::stringstream ss;
        ss << object.class_name.data << ":" << convertFloatToString(object.confidence) << "%";
        return ss.str();
    }

    inline void handleObject(Target &target, const Detection &object, cv::Mat &image){
        constructTarget(target, object);
        if (debug_contour_()){
            drawTarget(image, object, 3, cv::Scalar(0, 0, 255));
        }
        objects_.push_back(target);
    }
};

} //proc_image_processing


#endif //PROC_IMAGE_PROCESSING_DEEP_DICE_H
;

