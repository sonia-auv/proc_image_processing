/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include <sonia_common/ros/service_server_manager.h>
#include <opencv2/opencv.hpp>
#include <proc_image_processing/cpu/config.h>

void run();

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
    }else if (gpuCount == -1){
        cout << "The CUDA driver is not installed, or is incompatible" << endl;
    }else{
        cout << "CUDA is fully supported" << endl;
    }

    return 0;
}
