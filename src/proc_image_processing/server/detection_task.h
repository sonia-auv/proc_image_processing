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

#ifndef PROVIDER_VISION_PROC_DETECTION_TASK_H_
#define PROVIDER_VISION_PROC_DETECTION_TASK_H_

#include <lib_atlas/ros/image_publisher.h>
#include <mutex>
#include "proc_image_processing/config.h"
#include "proc_image_processing/server/filterchain.h"
#include "lib_atlas/pattern/runnable.h"
#include "proc_image_processing/server/ImageProvider.h"

namespace proc_image_processing {

/**
 * DetectionTask class is responsible of taking the image from an acquisition
 * loop,
 * broadcast it on topic and apply the given filterchain.
 */
    class DetectionTask : private atlas::Runnable{
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        using Ptr = std::shared_ptr<DetectionTask>;

        static const std::string EXEC_TAG;

        //==========================================================================
        // P U B L I C   C / D T O R S

        explicit DetectionTask(const std::string &topic_name, Filterchain::Ptr filterchain,
                               const std::string &execution_name);

        virtual ~DetectionTask();

        //==========================================================================
        // P U B L I C   M E T H O D S

        void StartDetectionTask();

        void StopDetectionTask();

        /**
         * Change the image returned by the detection task to the result of a
         * specific filter.
         * This behavior is being handled by the filterchain so this method is
         * a wrapper around the filterchain method that set the observed filter.
         */
        void ChangeReturnImageToFilter(const size_t &index);

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

        const std::string &GetDetectionTaskName() const;

        const std::string &GetMediaName() const;

    protected:
        void PublishAllTarget();
        void PublishClientImage();
        bool PrepareImageForPublishing(cv::Mat &image);
        void Run() override;

    private:
        //==========================================================================
        // P R I V A T E   M E M B E R S

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

        proc_image_processing::GlobalParamHandler &param_handler_;

        std::mutex newest_image_mutex_;

        cv::Mat image_being_processed_;

        bool returning_original_image_;
    };

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
    inline Filterchain::Ptr DetectionTask::GetFilterchain() const {
      return filterchain_;
    }

//------------------------------------------------------------------------------
//
    inline const std::string &DetectionTask::GetDetectionTaskName() const {
      return detection_task_name_;
    }

//------------------------------------------------------------------------------
//
    inline const std::string &DetectionTask::GetMediaName() const {
      return topic_name_;
    }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_PROC_DETECTION_TASK_H_
