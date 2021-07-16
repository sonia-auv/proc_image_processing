/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_PROC_DETECTION_TASK_H_
#define PROC_IMAGE_PROCESSING_PROC_DETECTION_TASK_H_

#include <sonia_common/ros/image_publisher.h>
#include <mutex>
#include "proc_image_processing/cpu/config.h"
#include "filter_chain.h"
#include "sonia_common/pattern/runnable.h"
#include "image_provider.h"

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

        explicit DetectionTask(const std::string &topic_name, FilterChain::Ptr filter_chain,
                               const std::string &execution_name);

        ~DetectionTask() override;

        void start();

        void stop();

        /**
         * Change the image returned by the detection task to the result of a
         * specific filter.
         * This behavior is being handled by the filterchain so this method is
         * a wrapper around the filterchain method that set the observed filter.
         */
        void changeReturnImageToFilter(const size_t &index);

        /**
         * Change the image returned by the detection task to the filterchain returned
         * image.
         * This is the default behavior, the image returned is the result of the
         * whole pipeline of filters.
         */
        void changeReturnImageToFilterChain();

        /**
         * Change the image returned by the detection task to the original image.
         * If this parameters is enables, the image is not being processed and
         * the image that is being sent on the network is the original image
         * from the media.
         */
        void changeReturnImageToOrigin();

        FilterChain::Ptr getFilterChain() const;

        const std::string &getName() const;

        const std::string &getMediaName() const;

    protected:
        void publishAllTarget();

        void publishClientImage();

        bool prepareForPublishing(cv::Mat &image);

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

        FilterChain::Ptr filterchain_;

        GlobalParamHandler &param_handler_;

        std::mutex newest_image_mutex_;

        cv::Mat image_being_processed_;

        bool returning_original_image_;
    };

    inline FilterChain::Ptr DetectionTask::getFilterChain() const {
        return filterchain_;
    }

    inline const std::string &DetectionTask::getName() const {
        return detection_task_name_;
    }

    inline const std::string &DetectionTask::getMediaName() const {
        return topic_name_;
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_PROC_DETECTION_TASK_H_
