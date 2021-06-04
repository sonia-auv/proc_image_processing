#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>
#include <sonia_common/Republish.h>
#include <cstdlib>

class Images {
public:

  ros::ServiceServer service;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub;
  image_transport::Publisher pub;
  uint last_seq;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (last_seq < msg->header.seq) {
      last_seq = msg->header.seq;
      pub.publish(msg);
    }
  }
  Images(ros::NodeHandle nh) :nh_(), it_(nh_) {
    service = nh_.advertiseService("image_republisher_node/republish", &Images::republish, this);
  }




  bool republish(sonia_common::Republish::Request& req,
    sonia_common::Republish::Response& res) {
    if (sub != NULL) {
      sub.shutdown();
      pub.shutdown();
    }

    // Load transport plugin
    std::string in_transport = "compressed";
    char* val;
    val = std::getenv("ROS_IP");
    std::string ip_address = "";
    if (val == NULL) {
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

int main(int argc, char** argv) {
  char* val;
  val = std::getenv("ROS_IP");
  std::string ip_address = "";
  if (val == NULL) {
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
