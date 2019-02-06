/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>
#include <proc_image_processing/republish.h>
#include <cstdlib>

class Images
{
  public:

      ros::ServiceServer service;
      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber sub;
      image_transport::Publisher pub;
      uint last_seq;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      if(last_seq < msg->header.seq)
      {
        last_seq = msg->header.seq;
        pub.publish(msg);
      }
    }
      Images(ros::NodeHandle nh):nh_(), it_(nh_)
      {
        service = nh_.advertiseService("image_republisher_node/republish", &Images::republish, this);
      }




  bool republish(proc_image_processing::republish::Request &req,
                 proc_image_processing::republish::Response &res)
  {
    if(sub != NULL)
    {
      sub.shutdown();
      pub.shutdown();
    }

    // Load transport plugin
    std::string in_transport = "compressed";
    char * val;
    val = std::getenv("ROS_IP");
    std::string ip_address = "";
    if(val == NULL){
      ip_address = "127.0.0.1";
    }
    else {
      ip_address = std::string(val);
    }
    boost::replace_all(ip_address, ".", "");
    pub = it_.advertise(req.topic_name + "_" + ip_address, 1);

    // Use PublisherPlugin::publish as the subscriber callback
    sub = it_.subscribe(req.topic_name, 1, &Images::imageCallback, this, in_transport);
    return true;
  }

};

int main(int argc, char** argv)
{
    char * val;
    val = std::getenv("ROS_IP");
    std::string ip_address = "";
    if(val == NULL){
      ip_address = "127.0.0.1";
    }
    else {
      ip_address = std::string(val);
    }
    boost::replace_all(ip_address, ".", "");
    ros::init(argc, argv, "image_republisher_node_" + ip_address);
    ros::NodeHandle nh;
    Images image(nh);
    ros::spin();

    return 0;
}
