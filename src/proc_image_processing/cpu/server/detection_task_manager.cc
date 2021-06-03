/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include <proc_image_processing/cpu/server/detection_task_manager.h>
#include "ImageProvider.h"

namespace proc_image_processing {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
    DetectionTaskManager::DetectionTaskManager() {}

//------------------------------------------------------------------------------
//
    DetectionTaskManager::~DetectionTaskManager() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
    std::string DetectionTaskManager::StartDetectionTask(
            const std::string &topic_name, Filterchain::Ptr filterchain,
            const std::string &execution_name) {
      if (execution_name.empty() ) {
        throw std::invalid_argument("The detection task name is not valid");
      }
      if (topic_name.empty() ) {
        throw std::invalid_argument("The topic name is not valid");
      }
      if( !filterchain )
      {
        throw std::invalid_argument("The filterchain is empty is not valid");
      }
      // Makes sure we have no detection task of that name.
      DetectionTask::Ptr task = GetDetectionTask(execution_name);

      if (task == nullptr) {
        task = std::make_shared<DetectionTask>(topic_name, filterchain,
                                               execution_name);
        task->StartDetectionTask();
        detection_tasks_.push_back(task);
      } else {
        throw std::logic_error("This detection task already exist.");
      }
      ROS_INFO("DetectionTask is ready.");
      return task->GetDetectionTaskName();
    }

//------------------------------------------------------------------------------
//
    void DetectionTaskManager::StopDetectionTask(
            const std::string &execution_name) {
      auto detection_task = GetDetectionTask(execution_name);

      auto it = std::find(detection_tasks_.begin(), detection_tasks_.end(),
                          detection_task);
      if (it != detection_tasks_.end()) {
        (*it)->StopDetectionTask();
        detection_tasks_.erase(it);
        ROS_INFO("Detection task is stopped.");
      } else {
        throw std::invalid_argument("This detection taks does not exist");
      }
    }

//------------------------------------------------------------------------------
//
    std::vector<std::string> DetectionTaskManager::GetAllDetectionTasksName()
    const {
      std::vector<std::string> names;
      for (const auto &task : detection_tasks_) {
        names.push_back(task->GetDetectionTaskName());
      }
      return names;
    }

//------------------------------------------------------------------------------
//
    void DetectionTaskManager::ChangeReturnImageToFilter(const std::string &name,
                                                         const size_t &index) {
      GetDetectionTask(name)->ChangeReturnImageToFilter(index);
    }

//------------------------------------------------------------------------------
//
    void DetectionTaskManager::ChangeReturnImageToFilterchain(
            const std::string &name) {
      GetDetectionTask(name)->ChangeReturnImageToFilterchain();
    }

//------------------------------------------------------------------------------
//
    void DetectionTaskManager::ChangeReturnImageToOrigin(const std::string &name) {
      GetDetectionTask(name)->ChangeReturnImageToOrigin();
    }

//------------------------------------------------------------------------------
//
    size_t DetectionTaskManager::GetAllDetectionTasksCount() const {
      return detection_tasks_.size();
    }

//------------------------------------------------------------------------------
//
    Filterchain::Ptr DetectionTaskManager::GetFilterchainFromDetectionTask(
            const std::string &name) const {
      auto detectTsk = GetDetectionTask(name);
      if (detectTsk == nullptr) {
        return nullptr;
      }
      return detectTsk->GetFilterchain();
    }

//------------------------------------------------------------------------------
//
    DetectionTask::Ptr DetectionTaskManager::GetDetectionTask(
            const std::string &execution_name) const {
      for (const auto &task : detection_tasks_) {
        if (task->GetDetectionTaskName().compare(execution_name) == 0) {
          return task;
        }
      }
      return nullptr;
    }

//------------------------------------------------------------------------------
//
    std::vector<std::string> DetectionTaskManager::GetAllMediasName() const
    {

        ros::master::V_TopicInfo info;
        ros::master::getTopics(info);
        std::vector<std::string> image_topic;

        for(auto i : info)
        {
            if( i.datatype.find("sensor_msgs/Image") != std::string::npos)
            {
                image_topic.push_back(i.name);
            }
        }
    return image_topic;
    }


}  // namespace proc_image_processing
