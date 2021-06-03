/**
 * \file	SonarMapper.h
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

#ifndef PROC_IMAGE_PROCESSING_RAW_MAP_H_
#define PROC_IMAGE_PROCESSING_RAW_MAP_H_

#include <opencv/cv.h>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include "AsyncImagePublisher.h"
#include "BaseObjectMapperInterface.h"
#include "SubmarinePosition.h"

namespace proc_image_processing {

class SonarMapper : public BaseObjectMapperInterface
{
public:

  const int NB_PIXEL_BY_METER = 20;
  const int MAP_WIDTH_METER = 60;
  const int MAP_HEIGTH_METER = 30;
  const int MAX_SCANLINE = 50;

  SonarMapper(const SubmarinePosition &submarine_position,
              const ros::NodeHandlePtr &nh);

  // Override BaseObjectMapperInterface
  void GetMapObject(MapObjectVector &list) override;
  void ResetMapper() override;

  // Sonar input processing
  void AddScanlineToMap(const sensor_msgs::PointCloud2::ConstPtr &msg);
private:
  void ExtractNewPoint(const sensor_msgs::PointCloud2::ConstPtr &msg, int i,
                       float &intensity, float &x, float &y, float &z) const;

private:
  cv::Mat sonar_map_;
  MapObjectVector object_list_;
  const SubmarinePosition &submarine_position_;
  int scanline_count_;
  ros::Subscriber scanline_subscriber_;
  AsyncImagePublisher image_publisher_;
    const ros::NodeHandlePtr nh_;
};

//=============================================================================
//      INLINE FUNCTIONS

inline void SonarMapper::GetMapObject(MapObjectVector &list)
{
  list.clear();
  std::copy(object_list_.begin(), object_list_.end(), list.begin());
}

inline void SonarMapper::ResetMapper()
{
  sonar_map_.setTo(0);
  object_list_.clear();
}

inline void
SonarMapper::ExtractNewPoint(const sensor_msgs::PointCloud2::ConstPtr &msg, int i,
                             float &intensity, float &x, float &y, float &z) const {
  int step = i * msg->point_step;
  memcpy(&x, &msg->data[step + msg->fields[0].offset], sizeof(float));
  memcpy(&y, &msg->data[step + msg->fields[1].offset], sizeof(float));
  memcpy(&z, &msg->data[step + msg->fields[2].offset], sizeof(float));
  memcpy(&intensity, &msg->data[step + msg->fields[3].offset], sizeof(float));
}


}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_RAW_MAP_H_
