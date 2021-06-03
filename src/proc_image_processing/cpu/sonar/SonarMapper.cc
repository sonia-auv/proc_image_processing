/**
 * \file	SonarMapper.cc
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


#include "SonarMapper.h"

namespace proc_image_processing {

SonarMapper::SonarMapper(const SubmarinePosition &submarine_position,
                         const ros::NodeHandlePtr &nh)
    : sonar_map_(MAP_HEIGTH_METER*NB_PIXEL_BY_METER, MAP_WIDTH_METER*NB_PIXEL_BY_METER, CV_8UC1),
      object_list_(0),
      submarine_position_(submarine_position),
      scanline_count_(0),
      image_publisher_("sonar_map"),
      nh_(nh)
{
  std::string topic_name ("/provider_sonar/point_cloud2" );
  scanline_subscriber_ = nh_->subscribe(topic_name, 100, &SonarMapper::AddScanlineToMap, this);
}

void SonarMapper::AddScanlineToMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  scanline_count_ ++;

  Eigen::Affine3d transform;
  // The transform order is:
  // 1) rotate the pointcloud coordinate (from local heading to global angle)
  // 2) translate it to the submarine's position (from local to global position)
  // 3) Scale it to the map's pixel by meter (ex. 10 cm = 1 pixel)
  // 4) Transport it to the center of the map (since the Mat center is width/2 an cols/2)
  //
  // Eigen take operation from the left to the right
  //
  transform = Eigen::Translation<double, 3>(sonar_map_.cols/2,sonar_map_.rows/2,0)* //4
              Eigen::Scaling ((double)NB_PIXEL_BY_METER) *                          //3
              Eigen::Translation<double, 3>(submarine_position_.GetPosition()) *    //2
              submarine_position_.GetQuaternion();                                  //1

  float x = 0, y = 0, z = 0, intensity = 0;
  int max_size = (int)(msg->data.size() / msg->point_step);
  for (int i = 0; i < max_size; i++) {
    // Fetch new coordiates
    ExtractNewPoint(msg, i, intensity, x, y, z);
    // transform it in the image coordinate
    Eigen::Vector3d point_cloud_coords (x,y,z);
    Eigen::Vector3d image_coords = transform * point_cloud_coords;

    int image_x = std::round(image_coords.x()), image_y = std::round(image_coords.y());
    if(0 <= image_x && image_x < sonar_map_.cols &&
        0 <= image_y && image_y < sonar_map_.rows)
    {
      // Can do this because <at> function returns a reference.
      uchar &value = sonar_map_.at<uchar>(image_y, image_x);
      value = (uchar)( ( (uint16_t)(intensity * 255.0f * 10) + (uint16_t)value)/2 );
    }
  }

  if( scanline_count_ > MAX_SCANLINE )
  {
      cv::operator-=(sonar_map_,20);
      scanline_count_ = 0;
      image_publisher_.Publish(sonar_map_);
  }
}


}