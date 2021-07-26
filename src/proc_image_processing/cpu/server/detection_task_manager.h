#ifndef PROC_IMAGE_PROCESSING_SERVER_DETECTION_TASK_MANAGER_H_
#define PROC_IMAGE_PROCESSING_SERVER_DETECTION_TASK_MANAGER_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <proc_image_processing/cpu/server/detection_task.h>
#include <proc_image_processing/cpu/server/filter_chain.h>

namespace proc_image_processing {

    class DetectionTaskManager {
    public:
        using Ptr = std::shared_ptr<DetectionTaskManager>;

        DetectionTaskManager();

        ~DetectionTaskManager();

        std::string StartDetectionTask(const std::string &topic_name,
                                       const FilterChain::Ptr &filter_chain,
                                       const std::string &execution_name);

        void stopDetectionTask(const std::string &execution_name);

        /**
         * Get the name of all existing detection tasks in the system.
         *
         * An existing detection tasks has been explicitly created by the user.
         * It can be paused so all the detection tasks here may not be running.
         *
         * \return The name of all detection tasks in the system.
         */
        std::vector<std::string> getDetectionTasksNames() const;

        /**
         * Change the image returned by the detection task to the result of a
         * specific filter.
         * This behavior is being handled by the filterchain so this method is
         * a wrapper around the filterchain method that set the observed filter.
         */
        void changeReturnImageToFilter(const std::string &name, const size_t &index);

        /**
         * Change the image returned by the detection task to the filterchain returned
         * image.
         * This is the default behavior, the image returned is the result of the
         * whole pipeline of filters.
         */
        void changeReturnImageToFilterChain(const std::string &name);

        /**
         * Change the image returned by the detection task to the original image.
         * If this parameters is enables, the image is not being processed and
         * the image that is being sent on the network is the original image
         * from the media.
         */
        void changeReturnImageToOrigin(const std::string &name);

        /**
         * Get the number of all detection tasks in the system.
         *
         * This will iterate through all detection tasks list in order to have the
         * total count.
         *
         * \return The total count of all detection tasks.
         */
        size_t getDetectionTasksCount() const;

        static std::vector<std::string> getMediasNames();

        /**
         * Get the filter chain object from the detection task.
         */
        FilterChain::Ptr getFilterChainFromDetectionTask(const std::string &name) const;

    private:
        DetectionTask::Ptr getDetectionTask(const std::string &execution_name) const;

        std::vector<DetectionTask::Ptr> detection_tasks_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_SERVER_DETECTION_TASK_MANAGER_H_
