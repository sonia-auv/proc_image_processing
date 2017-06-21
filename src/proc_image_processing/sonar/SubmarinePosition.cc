/**
 * \file	SubmarinePosition.cc
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "SubmarinePosition.h"

namespace proc_image_processing {

SubmarinePosition::SubmarinePosition(const ros::NodeHandlePtr &nh):
    position_xyz_(0,0,0),
    orientation_rpy_(0,0,0),
    orientation_quaternion_(1,0,0,0),
    nh_(nh)
{
  nav_odometry_subscriber_ = nh_->subscribe("/proc_navigation/odom", 100, &SubmarinePosition::OdometryCallback, this);
}


}// namespace proc_mapping