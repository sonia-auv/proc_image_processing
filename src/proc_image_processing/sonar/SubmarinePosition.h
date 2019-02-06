/**
 * \file	SubmarinePosition.h
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

#ifndef PROC_IMAGE_PROCESSING_SUBMARINEPOSITION_H
#define PROC_IMAGE_PROCESSING_SUBMARINEPOSITION_H

#include <lib_atlas/maths.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>
namespace proc_image_processing {

class SubmarinePosition {
public:
  const size_t X = 0;
  const size_t Y = 1;
  const size_t Z = 2;
  const size_t ROLL = 0;
  const size_t PITCH = 1;
  const size_t YAW = 2;

  // Constructor
  SubmarinePosition(const ros::NodeHandlePtr &nh);


  // Return submarine's orientation. Specifying the type,
  // because of Eigen, since everything can multiply everything...
  Eigen::Matrix3d GetRotationMatrix() const;
  Eigen::Quaterniond GetQuaternion() const;
  // ROLL PITCH YAW
  Eigen::Vector3d GetEuler() const;
  // XYZ
  Eigen::Vector3d GetPosition() const;


private:
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odo_in);

private:

  // ROS
  ros::Subscriber nav_odometry_subscriber_;
  Eigen::Vector3d position_xyz_, orientation_rpy_;
  Eigen::Quaterniond orientation_quaternion_;
    ros::NodeHandlePtr nh_;
};


inline void
SubmarinePosition::OdometryCallback(const nav_msgs::Odometry::ConstPtr &odo_in) {
  position_xyz_[X] = odo_in->pose.pose.position.x;
  position_xyz_[Y] = odo_in->pose.pose.position.y;
  position_xyz_[Z] = odo_in->pose.pose.position.z;
  orientation_rpy_[ROLL] = odo_in->pose.pose.orientation.x;
  orientation_rpy_[PITCH] = odo_in->pose.pose.orientation.y;
  orientation_rpy_[YAW] = odo_in->pose.pose.orientation.z;

  orientation_quaternion_ = atlas::EulerToQuat(orientation_rpy_);

}

inline Eigen::Matrix3d SubmarinePosition::GetRotationMatrix() const {
  return atlas::QuatToRot(orientation_quaternion_);
}

inline Eigen::Quaterniond SubmarinePosition::GetQuaternion() const
{
  return orientation_quaternion_;
}

inline Eigen::Vector3d SubmarinePosition::GetEuler() const
{
  return orientation_rpy_;
}

inline Eigen::Vector3d SubmarinePosition::GetPosition() const
{
  return position_xyz_;
}


} // namespace proc_image_processing
#endif //PROC_IMAGE_PROCESSING_SUBMARINEPOSITION_H
