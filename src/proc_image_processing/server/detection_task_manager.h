/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_SERVER_DETECTION_TASK_MANAGER_H_
#define PROVIDER_VISION_SERVER_DETECTION_TASK_MANAGER_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "proc_image_processing/server/detection_task.h"
#include "proc_image_processing/server/filterchain.h"

namespace proc_image_processing {

    class DetectionTaskManager {
    public:
        using Ptr = std::shared_ptr<DetectionTaskManager>;

        DetectionTaskManager();

        ~DetectionTaskManager();

        std::string StartDetectionTask(const std::string& topic_name,
            Filterchain::Ptr filterchain,
            const std::string& execution_name);

        void StopDetectionTask(const std::string& execution_name);

        /**
         * Get the name of all existing detection tasks in the system.
         *
         * An existing detection tasks has been explicitly created by the user.
         * It can be paused so all the detection tasks here may not be running.
         *
         * \return The name of all detection tasks in the system.
         */
        std::vector<std::string> GetAllDetectionTasksName() const;

        /**
         * Change the image returned by the detection task to the result of a
         * specific filter.
         * This behavior is being handled by the filterchain so this method is
         * a wrapper around the filterchain method that set the observed filter.
         */
        void ChangeReturnImageToFilter(const std::string& name, const size_t& index);

        /**
         * Change the image returned by the detection task to the filterchain returned
         * image.
         * This is the default behavior, the image returned is the result of the
         * whole pipeline of filters.
         */
        void ChangeReturnImageToFilterchain(const std::string& name);

        /**
         * Change the image returned by the detection task to the original image.
         * If this parameters is enables, the image is not being processed and
         * the image that is being sent on the network is the original image
         * from the media.
         */
        void ChangeReturnImageToOrigin(const std::string& name);

        /**
         * Get the number of all detection tasks in the system.
         *
         * This will iterate through all detection tasks list in order to have the
         * total count.
         *
         * \return The total count of all detection tasks.
         */
        size_t GetAllDetectionTasksCount() const;

        std::vector<std::string> GetAllMediasName() const;

        /**
         * Get the filter chain object from the detection task.
         */
        Filterchain::Ptr GetFilterchainFromDetectionTask(
            const std::string& name) const;

    private:
        DetectionTask::Ptr GetDetectionTask(const std::string& execution_name) const;

        std::vector<DetectionTask::Ptr> detection_tasks_;
    };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_SERVER_DETECTION_TASK_MANAGER_H_
