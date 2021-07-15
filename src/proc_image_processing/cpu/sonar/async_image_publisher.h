/**
 * \file	AsyncImagePublisher.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	27/07/2016
 */

#ifndef PROC_IMAGE_PROCESSING_ASYNC_IMAGE_PUBLISHER_H_
#define PROC_IMAGE_PROCESSING_ASYNC_IMAGE_PUBLISHER_H_

#include <queue>

#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

class AsyncImagePublisher {
public:


    explicit AsyncImagePublisher(const std::string &topic_name);

    ~AsyncImagePublisher();


    void publish(const cv::Mat &image);

private:

    void threadFunction();


    std::string topic_name_;
    // Flag to stop the thread
    bool stop_thread_;
    // The thread for broadcasting an image
    std::thread thread_;
    // Necessary publisher for the image
    image_transport::Publisher image_publisher_;
    image_transport::ImageTransport it_;

    std::queue<cv::Mat> images_to_publish_;
    std::mutex image_queue_mutex_;
};

#endif  // PROC_IMAGE_PROCESSING_ASYNC_IMAGE_PUBLISHER_H_
