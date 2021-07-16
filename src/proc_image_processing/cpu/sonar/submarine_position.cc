/**
 * \file	SubmarinePosition.cc
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
 */

#include "submarine_position.h"

#include <utility>

namespace proc_image_processing {

    SubmarinePosition::SubmarinePosition(ros::NodeHandlePtr nh) :
            position_xyz_(0, 0, 0),
            orientation_rpy_(0, 0, 0),
            orientation_quaternion_(1, 0, 0, 0),
            nh_(std::move(nh)) {
        nav_odometry_subscriber_ = nh_->subscribe("/proc_navigation/odom", 100, &SubmarinePosition::callbackOdometry,
                                                  this);
    }


}// namespace proc_mapping