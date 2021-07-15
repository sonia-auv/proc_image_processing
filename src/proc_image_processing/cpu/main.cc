/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>

#include <sonia_common/ros/service_server_manager.h>
#include <proc_image_processing/cpu/sonar/submarine_position.h>
#include <proc_image_processing/cpu/sonar/sonar_mapper.h>
#include <proc_image_processing/cpu/server/vision_server.h>

int main(int argc, char **argv) {
    ROS_INFO("Starting proc_image_processing (CPU mode)...");
    ros::init(argc, argv, "proc_image_processing");

    ros::NodeHandle nh("~");

    proc_image_processing::VisionServer pv(nh);

    ros::NodeHandlePtr nhp(&nh);
    proc_image_processing::SubmarinePosition sp(nhp);
    proc_image_processing::SonarMapper sonarMapper(sp, nhp);

    while (ros::ok()) {
        usleep(20000);
        ros::spinOnce();
    }

    return 0;
}