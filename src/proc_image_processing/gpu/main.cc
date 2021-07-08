/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include <sonia_common/ros/service_server_manager.h>
#include <proc_image_processing/cpu/sonar/SubmarinePosition.h>
#include <proc_image_processing/cpu/sonar/SonarMapper.h>
#include <proc_image_processing/cpu/server/vision_server.h>

int main(int argc, char **argv) {
    ROS_INFO("Starting proc_image_processing (GPU mode)...");

    ros::init(argc, argv, "proc_image_processing");
    ros::NodeHandle nh("~");

    int gpuCount = cv::cuda::getCudaEnabledDeviceCount();
    if (gpuCount == 0 || gpuCount == -1) {
        if (gpuCount == 0) ROS_ERROR("OpenCV is not compiled with cuda support. Running CPU mode instead...");
        else if (gpuCount == -1)
            ROS_ERROR("The CUDA driver is not installed, or is incompatible. Running CPU mode instead...");

        proc_image_processing::VisionServer pv(nh);

        ros::NodeHandlePtr nhp(&nh);
        proc_image_processing::SubmarinePosition sp(nhp);
        proc_image_processing::SonarMapper sonarMapper(sp, nhp);
    } else {
        ROS_INFO("proc_image_processing started with CUDA fully supported!");
        // TODO Launch GPU server
    }

    while (ros::ok()) {
        usleep(20000);
        ros::spinOnce();
    }

    return 0;
}
