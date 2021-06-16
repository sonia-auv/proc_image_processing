/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_PROC_DETECTION_TASK_H_
#define PROVIDER_VISION_PROC_DETECTION_TASK_H_

#include <sonia_common/ros/image_publisher.h>
#include <mutex>
#include "proc_image_processing/config.h"
#include "proc_image_processing/server/filterchain.h"
#include "sonia_common/pattern/runnable.h"
#include "proc_image_processing/server/ImageProvider.h"

namespace proc_image_processing {

  /**
   * DetectionTask class is responsible of taking the image from an acquisition
   * loop,
   * broadcast it on topic and apply the given filterchain.
   */
  class DetectionTask : private sonia_common::Runnable {
  public:
    using Ptr = std::shared_ptr<DetectionTask>;

    static const std::string EXEC_TAG;

    explicit DetectionTask(const std::string& topic_name, Filterchain::Ptr filterchain,
      const std::string& execution_name);

    virtual ~DetectionTask();

    void StartDetectionTask();

    void StopDetectionTask();

    /**
     * Change the image returned by the detection task to the result of a
     * specific filter.
     * This behavior is being handled by the filterchain so this method is
     * a wrapper around the filterchain method that set the observed filter.
     */
    void ChangeReturnImageToFilter(const size_t& index);

    /**
     * Change the image returned by the detection task to the filterchain returned
     * image.
     * This is the default behavior, the image returned is the result of the
     * whole pipeline of filters.
     */
    void ChangeReturnImageToFilterchain();

    /**
     * Change the image returned by the detection task to the original image.
     * If this parameters is enables, the image is not being processed and
     * the image that is being sent on the network is the original image
     * from the media.
     */
    void ChangeReturnImageToOrigin();

    Filterchain::Ptr GetFilterchain() const;

    const std::string& GetDetectionTaskName() const;

    const std::string& GetMediaName() const;

  protected:
    void PublishAllTarget();
    void PublishClientImage();
    bool PrepareImageForPublishing(cv::Mat& image);
    void Run() override;

  private:
    std::string detection_task_name_;
    std::string topic_name_;

    image_transport::Publisher image_publisher_;
    image_transport::ImageTransport it_;

    /**
     * This publisher will send the result of the image processing of the
     * filterchain onto the ROS pipeline. It will contain informations about
     * a potentially found object.
     *
     * This is a simple string separated by comma in this way:
     * obstacle_name:x,y,width,height,specific_message;
     * This could also return several objects this way:
     * obstacle_name:x,y,width,height,specific_message;
     * x2,y2,width2,height2,specific_message2;
     */
    ros::Publisher result_publisher_;

    ImageProvider image_provider_;

    Filterchain::Ptr filterchain_;

    proc_image_processing::GlobalParamHandler& param_handler_;

    std::mutex newest_image_mutex_;

    cv::Mat image_being_processed_;

    bool returning_original_image_;
  };

  inline Filterchain::Ptr DetectionTask::GetFilterchain() const {
    return filterchain_;
  }

  inline const std::string& DetectionTask::GetDetectionTaskName() const {
    return detection_task_name_;
  }

  inline const std::string& DetectionTask::GetMediaName() const {
    return topic_name_;
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_PROC_DETECTION_TASK_H_
