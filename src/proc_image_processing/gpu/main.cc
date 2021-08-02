#include <sonia_common/ros/service_server_manager.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    ROS_INFO("Starting proc_image_processing (GPU mode)...");

    ros::init(argc, argv, "proc_image_processing");

    int gpuCount = cv::cuda::getCudaEnabledDeviceCount();
    if (gpuCount == 0 || gpuCount == -1) {
        if (gpuCount == 0) ROS_ERROR("OpenCV is not compiled with CUDA support. Please use CPU mode.");
        else
            ROS_ERROR("The CUDA driver is not installed, or is incompatible. Please fix it or use CPU mode.");
        return 1;
    } else {
        ROS_INFO("proc_image_processing started with CUDA fully supported!");
        // TODO Replace to launch GPU server
        ROS_ERROR("GPU Mode has not been implemented yet. Please use CPU mode for the time being.");
        return 1;
    }

    return 0;
}
