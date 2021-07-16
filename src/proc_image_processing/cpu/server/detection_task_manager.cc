/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include <proc_image_processing/cpu/server/detection_task_manager.h>
#include "image_provider.h"

namespace proc_image_processing {

    DetectionTaskManager::DetectionTaskManager() {}

    DetectionTaskManager::~DetectionTaskManager() {}

    std::string DetectionTaskManager::StartDetectionTask(
            const std::string &topic_name, FilterChain::Ptr filter_chain,
            const std::string &execution_name) {
        if (execution_name.empty()) {
            throw std::invalid_argument("The detection task name is not valid");
        }
        if (topic_name.empty()) {
            throw std::invalid_argument("The topic name is not valid");
        }
        if (!filter_chain) {
            throw std::invalid_argument("The filter_chain is empty is not valid");
        }
        // Makes sure we have no detection task of that name.
        DetectionTask::Ptr task = getDetectionTask(execution_name);

        if (task == nullptr) {
            task = std::make_shared<DetectionTask>(topic_name, filter_chain,
                                                   execution_name);
            task->start();
            detection_tasks_.push_back(task);
        } else {
            throw std::logic_error("This detection task already exist.");
        }
        ROS_INFO("DetectionTask is ready.");
        return task->getName();
    }

    void DetectionTaskManager::stopDetectionTask(
            const std::string &execution_name) {
        auto detection_task = getDetectionTask(execution_name);

        auto it = std::find(detection_tasks_.begin(), detection_tasks_.end(),
                            detection_task);
        if (it != detection_tasks_.end()) {
            (*it)->stop();
            detection_tasks_.erase(it);
            ROS_INFO("Detection task is stopped.");
        } else {
            throw std::invalid_argument("This detection taks does not exist");
        }
    }

    std::vector<std::string> DetectionTaskManager::getDetectionTasksNames()
    const {
        std::vector<std::string> names;
        for (const auto &task : detection_tasks_) {
            names.push_back(task->getName());
        }
        return names;
    }

    void DetectionTaskManager::changeReturnImageToFilter(const std::string &name,
                                                         const size_t &index) {
        getDetectionTask(name)->changeReturnImageToFilter(index);
    }

    void DetectionTaskManager::changeReturnImageToFilterChain(
            const std::string &name) {
        getDetectionTask(name)->changeReturnImageToFilterChain();
    }

    void DetectionTaskManager::changeReturnImageToOrigin(const std::string &name) {
        getDetectionTask(name)->changeReturnImageToOrigin();
    }

    size_t DetectionTaskManager::getDetectionTasksCount() const {
        return detection_tasks_.size();
    }

    FilterChain::Ptr DetectionTaskManager::getFilterChainFromDetectionTask(
            const std::string &name) const {
        auto detectTsk = getDetectionTask(name);
        if (detectTsk == nullptr) {
            return nullptr;
        }
        return detectTsk->getFilterChain();
    }

    DetectionTask::Ptr DetectionTaskManager::getDetectionTask(
            const std::string &execution_name) const {
        for (const auto &task : detection_tasks_) {
            if (task->getName().compare(execution_name) == 0) {
                return task;
            }
        }
        return nullptr;
    }

    std::vector<std::string> DetectionTaskManager::getMediasNames() const {

        ros::master::V_TopicInfo info;
        ros::master::getTopics(info);
        std::vector<std::string> image_topic;

        for (auto i : info) {
            if (i.datatype.find("sensor_msgs/Image") != std::string::npos) {
                image_topic.push_back(i.name);
            }
        }
        return image_topic;
    }

}  // namespace proc_image_processing
