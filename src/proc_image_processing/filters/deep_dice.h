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
#include <sonia_common/Detection.h>
#include <sonia_common/DetectionArray.h>



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
                dice6_("dice6", true, &parameters_),
                roulette_("roulette", true, &parameters_),
                path_("path", true, &parameters_),
                cashinred_("cashinred", true, &parameters_),
                cashingreend_("cashinreen", true, &parameters_),
                slot_machine_("slot_machine", true, &parameters_),
                dollar_sign_("dollar_sign", true, &parameters_),
		bat_("bat", true, &parameters_),
		wolf_("wolf", true, &parameters_),
                color_(0,0,0)
        {
            image_subscriber_= ros::NodeHandle("~").subscribe("/deep_detector/bounding_box", 100, &DeepDice::boundingBoxCallback, this);
            SetName("DeepDice");
        };

        virtual ~DeepDice() {image_subscriber_.shutdown();}

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
                        color_ = cv::Scalar(0,0,255);
                        handleObject(target, object, image, color_);
                    }
                    if(dice2_.GetValue() && object.class_name.data == dice2_.GetName())  {
                        color_ = cv::Scalar(0,255,0);
                        handleObject(target, object, image, color_);
                    }
                    if(dice5_.GetValue() && object.class_name.data == dice5_.GetName())  {
                        color_ = cv::Scalar(255,0,0);
                        handleObject(target, object, image, color_);
                    }
                    if(dice6_.GetValue() && object.class_name.data == dice6_.GetName())  {
                        color_ = cv::Scalar(244,185,66);
                        handleObject(target, object, image, color_);
                    }
                    if(roulette_.GetValue() && object.class_name.data == roulette_.GetName())  {
                        color_ = cv::Scalar(200,185,66);
                        handleObject(target, object, image, color_);
                    }
                    if(path_.GetValue() && object.class_name.data == path_.GetName())  {
                        color_ = cv::Scalar(244,185,100);
                        handleObject(target, object, image, color_);
                    }
                    if(cashinred_.GetValue() && object.class_name.data == cashinred_.GetName())  {
                        color_ = cv::Scalar(244,185,200);
                        handleObject(target, object, image, color_);
                    }
                    if(cashingreend_.GetValue() && object.class_name.data == cashingreend_.GetName())  {
                        color_ = cv::Scalar(244,200,66);
                        handleObject(target, object, image, color_);
                    }
                    if(slot_machine_.GetValue() && object.class_name.data == slot_machine_.GetName())  {
                        color_ = cv::Scalar(244,200,66);
                        handleObject(target, object, image, color_);
                    }if(dollar_sign_.GetValue() && object.class_name.data == dollar_sign_.GetName())  {
                        color_ = cv::Scalar(147, 145, 0);
                        handleObject(target, object, image, color_);
                    }if(bat_.GetValue() && object.class_name.data == bat_.GetName())  {
                        color_ = cv::Scalar(217, 244, 66);
                        handleObject(target, object, image, color_);
                    }if(wolf_.GetValue() && object.class_name.data == wolf_.GetName())  {
                        color_ = cv::Scalar(66, 244, 223);
                        handleObject(target, object, image, color_);
                    }
                }

                for (int i = 0; i < (int)objects_.size(); ++i) {
                    NotifyTarget(objects_.back());
                    objects_.pop_back();
                }
            }
        };


    private:
        const int BBOX_INFO_RECT_HEIGHT = 30;
        const int BBOX_X_TOP_LEFT_CORRECTION = 5;
        const int BBOX_X_BOTTOM_RIGHT_CORRECTION = 10;
        const cv::Scalar BBOX_INFO_TEXT_COLOR = cv::Scalar(255,255,255);
        const int BBOX_INFO_FONT = cv::FONT_HERSHEY_TRIPLEX;

        ros::Subscriber image_subscriber_;
        ros::NodeHandle nh_;
        std::vector<sonia_common::Detection> bounding_box_;
        std::vector<Target> objects_;
        Parameter<bool> enable_, debug_contour_, dice1_, dice2_, dice5_, dice6_, roulette_, path_, cashinred_, cashingreend_, slot_machine_, dollar_sign_, bat_,wolf_;
        int image_width_;
        int image_height_;
        cv::Scalar color_;


        void boundingBoxCallback(const sonia_common::DetectionArrayConstPtr &msg){
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



        inline void constructTarget(Target &target, const sonia_common::Detection &object){
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

        inline void drawTarget(cv::Mat &image, const sonia_common::Detection &object, int thickness=3, const cv::Scalar &color_box=cv::Scalar(0, 255, 0)){
            int origin_x = (int)(object.bbox.center.x - (object.bbox.size_x/2));
            int origin_y = (int)(object.bbox.center.y - (int)(object.bbox.size_y/2));

            int top_left_x = origin_x - BBOX_X_TOP_LEFT_CORRECTION;
            int top_left_y = origin_y - BBOX_INFO_RECT_HEIGHT;
            int bottom_right_x = (int)object.bbox.size_x + BBOX_X_BOTTOM_RIGHT_CORRECTION;
            int bottom_right_y = BBOX_INFO_RECT_HEIGHT;

            cv::Rect rect_top(top_left_x, top_left_y, bottom_right_x, bottom_right_y);
            cv::Rect rect(origin_x, origin_y, (int)object.bbox.size_x, (int)object.bbox.size_y);

            cv::rectangle(image, rect, color_box, thickness);
            cv::rectangle(image, rect_top, color_box, CV_FILLED);

            std::string text = creatTextBoundingBox(object);
            cv::putText(image, text, cv::Point(origin_x,origin_y), BBOX_INFO_FONT, 1, BBOX_INFO_TEXT_COLOR, 2, CV_AA);
        }

        inline std::string creatTextBoundingBox(const sonia_common::Detection &object){
            std::stringstream ss;
            ss << object.class_name.data << ":" << convertFloatToString(object.confidence) << "%";
            return ss.str();
        }

        inline void handleObject(Target &target, const sonia_common::Detection &object, cv::Mat &image, const cv::Scalar &color){
            constructTarget(target, object);
            if (debug_contour_()){
                drawTarget(image, object, 10, color);
            }
            objects_.push_back(target);
        }


    };

} //proc_image_processing


#endif //PROC_IMAGE_PROCESSING_DEEP_DICE_H
;

