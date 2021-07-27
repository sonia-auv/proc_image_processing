/// \author jeremie
/// \date 2/27/17


#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <thread>
#include "sonia_common/ros/image_publisher.h"
#include "proc_image_processing/cpu/server/vision_server.h"
#include "proc_image_processing/cpu/config.h"

ros::NodeHandle *nhp;

class ImagePublishingThread {
public:
    explicit ImagePublishingThread(const std::string &topic_name) :
            image_publisher_(),
            exit_thread_(false),
            pause_thread_(false),
            thread_([this] { PublishingThread(); }),
            it_(*nhp) {
        image_publisher_ = it_.advertise(topic_name, 100);
    }

    ~ImagePublishingThread() {
        StopThread();
        thread_.join();
        image_publisher_.shutdown();
    }

    inline void StopThread() { exit_thread_ = true; }
    inline void PauseThread() { pause_thread_ = true; }
    inline void RestartThread() { pause_thread_ = false; }
private:
    void PublishingThread() {
        cv::Mat image_to_publish(400, 400, CV_8UC1);
        uint8_t i = 0;
        cv_bridge::CvImage ros_image;
        while (!exit_thread_) {
            while (pause_thread_);
            // Publish a new image.
            image_to_publish.setTo(i);
            i++;
            ros_image.image = image_to_publish;
            ros_image.encoding = sensor_msgs::image_encodings::MONO8;
            image_publisher_.publish(ros_image.toImageMsg());
            // 30 ms
            usleep(30000);
        }
    }
    bool exit_thread_, pause_thread_;
    std::thread thread_;
    image_transport::Publisher image_publisher_;
    image_transport::ImageTransport it_;
};

void VisionServerThread() {
    proc_image_processing::VisionServer visionServer(*nhp);

    while (ros::ok()) {
        usleep(20000);
        ros::spinOnce();
    }
}

TEST(BlackBoxTest, test) {

    std::thread vsthd(&VisionServerThread);

    std::string base_node_name = proc_image_processing::kRosNodeName;
    std::string exec_cmd_name = base_node_name + std::string("execute_cmd");
    std::string list_name = base_node_name + std::string("get_information_list");
    std::string media_exec_name = base_node_name + std::string("get_media_from_execution");
    ros::ServiceClient execute_service = nhp->serviceClient<sonia_common::ExecuteCmd>(exec_cmd_name);
    ros::ServiceClient list_service = nhp->serviceClient<sonia_common::GetInformationList>(list_name);
    ros::ServiceClient media_service = nhp->serviceClient<sonia_common::GetMediaFromExecution>(media_exec_name);

ImagePublishingThread thread_1("/provider_camera/test1"), thread_2("/provider_camera/test2");

    // Make sure the feed are seen by the system
    sonia_common::GetInformationListRequest informationListRequest;
    sonia_common::GetInformationListResponse informationListResponse;
    informationListRequest.cmd = informationListRequest.MEDIA;
    list_service.call(informationListRequest, informationListResponse);

    ASSERT_NE(informationListResponse.list.find("/provider_camera/test1"), -1);
    ASSERT_NE(informationListResponse.list.find("/provider_camera/test2"), -1);

    // Start an execution
    sonia_common::ExecuteCmdRequest executeCmdRequest;
    sonia_common::ExecuteCmdResponse executeCmdResponse;
    executeCmdRequest.cmd = executeCmdRequest.START;
    executeCmdRequest.media_name = "/provider_camera/test1";
    executeCmdRequest.node_name = "Testouille1";
    executeCmdRequest.filterchain_name = "simple_buoy_green";
    execute_service.call(executeCmdRequest, executeCmdResponse);
    ASSERT_NE(executeCmdResponse.response.find("Testouille1"), -1);

    // Try to start an exceution with same name
    execute_service.call(executeCmdRequest, executeCmdResponse);
    ASSERT_EQ(executeCmdResponse.response.find("Testouille1"), -1);


    // Start another execution
    executeCmdRequest.media_name = "/provider_camera/test2";
    executeCmdRequest.node_name = "Testouille2";
    execute_service.call(executeCmdRequest, executeCmdResponse);
    ASSERT_NE(executeCmdResponse.response.find("Testouille2"), -1);

    informationListRequest.cmd = informationListRequest.EXEC;
    list_service.call(informationListRequest, informationListResponse);

    ASSERT_NE(informationListResponse.list.find("Testouille1"), -1);
    ASSERT_NE(informationListResponse.list.find("Testouille2"), -1);

    // end an execution
    executeCmdRequest.cmd = executeCmdRequest.STOP;
    executeCmdRequest.media_name = "/provider_camera/test";
    executeCmdRequest.node_name = "Testouille1";
    execute_service.call(executeCmdRequest, executeCmdResponse);

    informationListRequest.cmd = informationListRequest.EXEC;
    list_service.call(informationListRequest, informationListResponse);

    ASSERT_EQ(informationListResponse.list.find("Testouille1"), -1);
    ASSERT_NE(informationListResponse.list.find("Testouille2"), -1);

    // Try to end it again
    execute_service.call(executeCmdRequest, executeCmdResponse);

    // test as much as you want here...
    sleep(3600);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "proc_image_processing");
    nhp = new ros::NodeHandle{ "~" };
    return RUN_ALL_TESTS();
}
