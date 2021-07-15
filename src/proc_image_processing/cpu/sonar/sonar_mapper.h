/**
 * \file	SonarMapper.h
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
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
#include "async_image_publisher.h"
#include "base_object_mapper.h"
#include "submarine_position.h"

namespace proc_image_processing {

    class SonarMapper : public BaseObjectMapperInterface {
    public:

        const int NB_PIXEL_BY_METER = 20;
        const int MAP_WIDTH_METER = 60;
        const int MAP_HEIGHT_METER = 30;
        const int MAX_SCANLINE = 50;

        SonarMapper(const SubmarinePosition &submarine_position, const ros::NodeHandlePtr &nh);

        // Override BaseObjectMapperInterface
        void getMapObject(MapObjectVector &list) override;

        void resetMapper() override;

        // Sonar input processing
        void addScanLineToMap(const sensor_msgs::PointCloud2::ConstPtr &msg);

    private:
        void extractNewPoint(const sensor_msgs::PointCloud2::ConstPtr &msg, int i,
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

    inline void SonarMapper::getMapObject(MapObjectVector &list) {
        list.clear();
        std::copy(object_list_.begin(), object_list_.end(), list.begin());
    }

    inline void SonarMapper::resetMapper() {
        sonar_map_.setTo(0);
        object_list_.clear();
    }

    inline void SonarMapper::extractNewPoint(const sensor_msgs::PointCloud2::ConstPtr &msg, int i,
                                             float &intensity, float &x, float &y, float &z) const {
        int step = i * msg->point_step;
        memcpy(&x, &msg->data[step + msg->fields[0].offset], sizeof(float));
        memcpy(&y, &msg->data[step + msg->fields[1].offset], sizeof(float));
        memcpy(&z, &msg->data[step + msg->fields[2].offset], sizeof(float));
        memcpy(&intensity, &msg->data[step + msg->fields[3].offset], sizeof(float));
    }


}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_RAW_MAP_H_
