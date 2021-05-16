/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include <sonia_common/ros/service_server_manager.h>
#include <ros/ros.h>
#include <proc_image_processing/sonar/SubmarinePosition.h>
#include <proc_image_processing/sonar/SonarMapper.h>
#include <proc_image_processing/server/vision_server.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>

void run();
void runCUDA();

using namespace std;

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
    cout << "OpenCV: " << CV_VERSION << endl;
    cout << "OpenCV Major version: " << CV_MAJOR_VERSION << endl;
    cout << "OpenCV Minor version: " << CV_MINOR_VERSION << endl;
    cout << "OpenCV Subminor version: " << CV_SUBMINOR_VERSION << endl;

    ros::init(argc, argv, "proc_image_processing");
    int gpuCount = cv::cuda::getCudaEnabledDeviceCount();
    if (gpuCount == 0){
        cout << "OpenCV is not compiled with cuda support" << endl;
        run();
    }else if (gpuCount == -1){
        cout << "The CUDA driver is not installed, or is incompatible" << endl;
        run();
    }else{
        cout << "CUDA is fully supported";
        runCUDA();
    }

    return 0;
}

void run() {
    ros::NodeHandle nh("~");

    proc_image_processing::VisionServer pv(nh);

    ros::NodeHandlePtr nhp(&nh);
    proc_image_processing::SubmarinePosition sp(nhp);
    proc_image_processing::SonarMapper sonarMapper(sp, nhp);

    while (ros::ok()) {
        usleep(20000);
        ros::spinOnce();
    }
}

void runCUDA(){
    // TODO Start CUDA enabled vision server with future implementation of CUDA image filters
}
