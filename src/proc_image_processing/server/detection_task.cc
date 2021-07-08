/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include "proc_image_processing/server/detection_task.h"
#include <ros/ros.h>
#include <sonia_common/VisionTarget.h>
#include <std_msgs/String.h>
#include "proc_image_processing/server/target.h"

namespace proc_image_processing {

  const std::string DetectionTask::EXEC_TAG = "[EXECUTION]";

  DetectionTask::DetectionTask(const std::string& topic_name,
    Filterchain::Ptr filterchain,
    const std::string& execution_name)
    : detection_task_name_(execution_name),
    topic_name_(topic_name),
    image_publisher_(),
    it_(ros::NodeHandle("~")),
    result_publisher_(),
    image_provider_(topic_name),
    filterchain_(filterchain),
    param_handler_(filterchain_->GetParameterHandler()),
    returning_original_image_(false) {
    result_publisher_ = ros::NodeHandle().advertise<sonia_common::VisionTarget>(
      kRosNodeName + detection_task_name_ + "_result", 50);

    image_publisher_ = it_.advertise(detection_task_name_ + "_image", 100);
  }

  DetectionTask::~DetectionTask() {
    if (IsRunning()) {
      StopDetectionTask();
    }
    ROS_INFO("%s Destroying execution", EXEC_TAG.c_str());
  }

  void DetectionTask::StartDetectionTask() {
    if (IsRunning()) {
      throw std::logic_error("This detection task is already running.");
    }
    Start();
    ROS_INFO("Starting detection task %s", this->detection_task_name_.c_str());
  }

  void DetectionTask::StopDetectionTask() {
    if (!IsRunning()) {
      throw std::logic_error("This detection task is not running.");
    }
    Stop();
    result_publisher_.shutdown();
    image_publisher_.shutdown();
  }

  void DetectionTask::ChangeReturnImageToFilter(const size_t& index) {
    returning_original_image_ = false;
    filterchain_->SetObserver(index);
  }

  void DetectionTask::ChangeReturnImageToFilterchain() {
    returning_original_image_ = false;
    auto last_index = filterchain_->GetAllFilters().size() - 1;
    ChangeReturnImageToFilter(last_index);
  }

  void DetectionTask::ChangeReturnImageToOrigin() {
    returning_original_image_ = true;
  }

  void DetectionTask::Run() {
    unsigned int image_id_old = -1, image_id_new = -1;
    while (!MustStop()) {
      try {
        // Fetch the image.
        image_provider_.GetImage(image_being_processed_, image_id_new);
        // if same ID as previous image, do not reprocess it.
        if (image_id_old == image_id_new) {
          usleep(1000);
          continue;
        }
        else {
          image_id_old = image_id_new;
        }
        // Process the image
        filterchain_->ExecuteFilterChain(image_being_processed_);
        // Send the image of the filterchain
        PublishClientImage();
        // Send the target
        PublishAllTarget();
        // Clear them to have fresh one on next execution
        param_handler_.clearTarget();
      }
      catch (std::exception& e) {
        ROS_ERROR("Catched error in detection task %s, %s", detection_task_name_.c_str(), e.what());
      }
    }
  }

  void DetectionTask::PublishAllTarget() {
    // All the target are in the param_handler's queue
    proc_image_processing::TargetQueue targetQueue = param_handler_.getTargetQueue();
    sonia_common::VisionTarget msg;
    while (!targetQueue.empty()) {
      // Set the message and delete the element
      targetQueue.front().SetMessage(msg);
      targetQueue.pop();
      // Send it to ROS
      result_publisher_.publish(msg);
    }
  }

  void DetectionTask::PublishClientImage() {
    cv::Mat image_to_pubish;
    cv_bridge::CvImage ros_image;
    // Get the image to publish, depending on client's demand
    if (returning_original_image_) {
      image_to_pubish = param_handler_.getOriginalImage();
    }
    else {
      image_to_pubish = image_being_processed_;
    }
    // Prepare it to be always the same format
    if (!PrepareImageForPublishing(image_to_pubish)) {
      ROS_ERROR("Detection task %s could not format image for client", detection_task_name_.c_str());
    }
    // Publish it
    try {
      ros_image.image = image_to_pubish;
      ros_image.encoding = sensor_msgs::image_encodings::BGR8;
      image_publisher_.publish(ros_image.toImageMsg());
    }
    catch (std::exception& e) {
      ROS_ERROR("Detection task %s could not publish processed image.", detection_task_name_.c_str());
    }
  }

  bool DetectionTask::PrepareImageForPublishing(cv::Mat& image) {
    if (image.empty())
      return false;
    // Ensure unsigned 256 bit
    if (image.depth() != CV_8U) {
      image.convertTo(image, CV_8U);
    }
    // Ensure 3 channel (RGB
    if (image.channels() == 1) {
      cv::cvtColor(image, image, CV_GRAY2BGR);
    }

    cv::resize(image, image, cv::Size(600, 400));
    return true;
  }

}  // namespace proc_image_processing
