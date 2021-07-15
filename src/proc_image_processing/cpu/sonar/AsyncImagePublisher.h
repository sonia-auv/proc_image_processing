/**
 * \file	AsyncImagePublisher.h
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


  void Publish(const cv::Mat &image);

 private:

  void ThreadFunction();


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
