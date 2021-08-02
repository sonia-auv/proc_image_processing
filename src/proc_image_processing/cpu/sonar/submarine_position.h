#ifndef PROC_IMAGE_PROCESSING_SUBMARINE_POSITION_H
#define PROC_IMAGE_PROCESSING_SUBMARINE_POSITION_H

#include <sonia_common/maths.h>
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
        explicit SubmarinePosition(ros::NodeHandlePtr nh);


        // Return submarine's orientation. Specifying the type,
        // because of Eigen, since everything can multiply everything...
        Eigen::Matrix3d getRotationMatrix() const;

        Eigen::Quaterniond getQuaternion() const;

        // ROLL PITCH YAW
        Eigen::Vector3d getEuler() const;

        // XYZ
        Eigen::Vector3d getPosition() const;


    private:
        void callbackOdometry(const nav_msgs::Odometry::ConstPtr &odo_in);

    private:

        // ROS
        ros::Subscriber nav_odometry_subscriber_;
        Eigen::Vector3d position_xyz_, orientation_rpy_;
        Eigen::Quaterniond orientation_quaternion_;
        ros::NodeHandlePtr nh_;
    };


    inline void
    SubmarinePosition::callbackOdometry(const nav_msgs::Odometry::ConstPtr &odo_in) {
        position_xyz_[X] = odo_in->pose.pose.position.x;
        position_xyz_[Y] = odo_in->pose.pose.position.y;
        position_xyz_[Z] = odo_in->pose.pose.position.z;
        orientation_rpy_[ROLL] = odo_in->pose.pose.orientation.x;
        orientation_rpy_[PITCH] = odo_in->pose.pose.orientation.y;
        orientation_rpy_[YAW] = odo_in->pose.pose.orientation.z;

        orientation_quaternion_ = sonia_common::EulerToQuat(orientation_rpy_);
    }

    inline Eigen::Matrix3d SubmarinePosition::getRotationMatrix() const {
        return sonia_common::QuatToRot(orientation_quaternion_);
    }

    inline Eigen::Quaterniond SubmarinePosition::getQuaternion() const {
        return orientation_quaternion_;
    }

    inline Eigen::Vector3d SubmarinePosition::getEuler() const {
        return orientation_rpy_;
    }

    inline Eigen::Vector3d SubmarinePosition::getPosition() const {
        return position_xyz_;
    }


} // namespace proc_image_processing
#endif //PROC_IMAGE_PROCESSING_SUBMARINE_POSITION_H
