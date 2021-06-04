/// \author	Jeremie 
/// \date 2/22/17


#ifndef PROC_IMAGE_PROCESSING_IMAGEPROVIDER_H
#define PROC_IMAGE_PROCESSING_IMAGEPROVIDER_H

#include <string>
#include <functional>
#include <vector>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class ImageProvider {
public:
  ImageProvider(const std::string& topic_name)
    :topic_name_(topic_name),
    it_(ros::NodeHandle("~")),
    image_id_(0) {
    std::string compressed = "/compressed";

    if (topic_name.compare(topic_name.length() - compressed.length(), compressed.length(), compressed) == 0) {
      image_transport::TransportHints hint("compressed");
      subscriber_ = it_.subscribe(topic_name.substr(0, topic_name.length() - compressed.length()), 50, &ImageProvider::ImageCallback, this, hint);
    }
    else {
      subscriber_ = it_.subscribe(topic_name, 50, &ImageProvider::ImageCallback, this);

    }

  }

  void ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    image_mutex_.lock();
    try {
      cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(image_);
      image_id_++;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("%s Could not convert from '%s' to 'bgr8'.", topic_name_.c_str(), msg->encoding.c_str());
    }
    image_mutex_.unlock();
  }

  inline void GetImage(cv::Mat& image, unsigned int& image_id) {
    image_mutex_.lock();
    image_.copyTo(image);
    image_id = image_id_;
    image_mutex_.unlock();
  }
private:
  std::string topic_name_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber subscriber_;
  cv::Mat image_;
  std::mutex image_mutex_;
  unsigned int image_id_;
};


#endif //PROC_IMAGE_PROCESSING_IMAGEPROVIDER_H
