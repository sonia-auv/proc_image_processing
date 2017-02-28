//
// Created by jeremie on 2/27/17.
//

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ros/ros.h>
#include <thread>
#include "lib_atlas/ros/image_publisher.h"
ros::NodeHandle *nhp;

class ImagePulishingThread
{
public:
    ImagePulishingThread(const std::string &topic_name):
            image_publisher_(),
            exit_thread_(false),
            pause_thread_(false),
            thread_(std::bind(&ImagePulishingThread::PublishingThread, this)),
            it_(*nhp)
    {
        image_publisher_ = it_.advertise(topic_name, 100);
    }
    ~ImagePulishingThread()
    {
        StopThread();
        thread_.join();
        image_publisher_.shutdown();
    }
    inline void StopThread(){exit_thread_ = true;}
    inline void PauseThread(){pause_thread_ = true;}
    inline void RestartThread(){pause_thread_ = false;}
private:
    void PublishingThread()
    {
        cv::Mat image_to_publish(400,400,CV_8UC1);
        uint8_t i = 0;
        cv_bridge::CvImage ros_image;
        while(!exit_thread_)
        {
            while(pause_thread_);
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

TEST(BlackBoxTest, test) {

    ImagePulishingThread thread_1("/provider_camera/test1"), thread_2("/provider_camera/test2");
    ros::master::V_TopicInfo info;
    ros::master::getTopics(info);
    std::vector<std::string> image_topic;

    for(auto i : info)
    {
        std::cout << i.name << " " << i.datatype << std::endl;
        if( i.datatype.find("sensor_msgs/Image") != -1)
        {
            image_topic.push_back(i.name);
        }
    }

    for( auto tmp : image_topic)
    {
        std::cout << tmp << std::endl;
    }
    while(1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "proc_image_processing");
    nhp = new ros::NodeHandle{"~"};
    return RUN_ALL_TESTS();
}
