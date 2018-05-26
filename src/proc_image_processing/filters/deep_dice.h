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
            debug_contour_("Debug contour", false, &parameters_)
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
            for (auto &object: bounding_box_) {
                constructTarget(target, object);
                if (debug_contour_()){
                    drawTarget(image, object, 3, cv::Scalar(0, 0, 255));
                }
                objects_.push_back(target);
            }

            for (int i = 0; i < objects_.size(); ++i) {
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
    Parameter<bool> enable_, debug_contour_;

    void boundingBoxCallback(const DetectionArrayConstPtr &msg){
        if (bounding_box_.empty())
            bounding_box_.clear();
        bounding_box_ = msg->detected_object;

    }

    std::string convertFloatToString(float value){
        std::ostringstream ss;
        ss << value;
        return ss.str();
    }

    void constructTarget(Target &target, const Detection &object){
        target.SetCenter((int)object.bbox.center.x, (int)object.bbox.center.y);
        target.SetSize((int)object.bbox.size_x, (int)object.bbox.size_y);
        target.SetSpecField_1(object.class_name.data);
        target.SetSpecField_2(convertFloatToString(object.confidence));
    }

    void drawTarget(cv::Mat &image, const Detection &object, int thickness=3, const cv::Scalar &color_box=cv::Scalar(0, 255, 0)){
        auto origin_x = (int)(object.bbox.center.x - (object.bbox.size_x/2));
        auto origin_y = (int)(object.bbox.center.y - (int)(object.bbox.size_y/2));
        cv::Rect rect(origin_x, origin_y, (int)object.bbox.size_x, (int)object.bbox.size_y);
        cv::rectangle(image, rect, color_box, thickness);
    }
};

} //proc_image_processing


#endif //PROC_IMAGE_PROCESSING_DEEP_DICE_H
