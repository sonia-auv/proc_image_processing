/**
 * \file	AsyncImagePublisher.cc
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	27/07/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "AsyncImagePublisher.h"
#include <cv_bridge/cv_bridge.h>

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
AsyncImagePublisher::AsyncImagePublisher(const std::string &topic_name)
    : topic_name_(topic_name),
      stop_thread_(false),
      thread_(),
      image_publisher_(),
      it_(ros::NodeHandle("~"))
{
  // Create the broadcast topic.
  image_publisher_ = it_.advertise(topic_name, 100);
  thread_ = std::thread(std::bind(&AsyncImagePublisher::ThreadFunction, this));
}

//------------------------------------------------------------------------------
//
AsyncImagePublisher::~AsyncImagePublisher() {
  // Set the flag to stop the thread and wait for it to stop
  stop_thread_ = true;
  thread_.join();
  // Shutdown the topic
  image_publisher_.shutdown();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void AsyncImagePublisher::Publish(const cv::Mat &image) {
  image_queue_mutex_.lock();
  images_to_publish_.push(image);
  image_queue_mutex_.unlock();
}

//------------------------------------------------------------------------------
//
void AsyncImagePublisher::ThreadFunction() {
  cv_bridge::CvImage ros_image;
  ROS_INFO("Starting ascyn image publisher");
  while (!stop_thread_) {
    image_queue_mutex_.lock();
    size_t size = images_to_publish_.size();
    image_queue_mutex_.unlock();
    // No image, wait a bit.
    if (size == 0) {
      // If no image, wait 1 ms
      usleep(1000);
    } else if (size > 10) {
      ROS_ERROR("Too much image to publish, clearing the buffer on %s",
                topic_name_.c_str());
      image_queue_mutex_.lock();
      // Clear the queue
      while (!images_to_publish_.empty()) {
        images_to_publish_.pop();
      }
      image_queue_mutex_.unlock();

    } else {
      image_queue_mutex_.lock();
      cv::Mat tmp_image(images_to_publish_.front());
      images_to_publish_.pop();
      image_queue_mutex_.unlock();
      ros_image.image = tmp_image;
      ros_image.encoding = sensor_msgs::image_encodings::MONO8;
      image_publisher_.publish(ros_image.toImageMsg());
    }
  }
  ROS_INFO("Closing ascyn image publisher");
}
