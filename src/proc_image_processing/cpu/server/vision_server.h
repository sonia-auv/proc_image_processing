#ifndef PROC_IMAGE_PROCESSING_SERVER_VISION_SERVER_H_
#define PROC_IMAGE_PROCESSING_SERVER_VISION_SERVER_H_

#include <sonia_common/CopyFilterchain.h>
#include <sonia_common/ExecuteCmd.h>
#include <sonia_common/GetFilterchainFilter.h>
#include <sonia_common/GetFilterchainFilterAllParam.h>
#include <sonia_common/GetFilterchainFilterParam.h>
#include <sonia_common/GetFilterchainFromExecution.h>
#include <sonia_common/GetInformationList.h>
#include <sonia_common/GetMediaFromExecution.h>
#include <sonia_common/ManageFilterchain.h>
#include <sonia_common/ManageFilterchainFilter.h>
#include <sonia_common/SaveFilterchain.h>
#include <sonia_common/SetFilterchainFilterObserver.h>
#include <sonia_common/SetFilterchainFilterOrder.h>
#include <sonia_common/SetFilterchainFilterParam.h>
#include <memory>
#include <string>
#include <vector>
#include "proc_image_processing/cpu/config.h"
#include "detection_task.h"
#include "detection_task_manager.h"
#include "filter_chain_manager.h"

namespace proc_image_processing {

    using namespace proc_image_processing;

    /**
     * Vision server is the main class of the system
     * It's job is to assemble and connect the pieces to create execution
     * It is the owner of the active DetectionTask and MediaStreamer
     * It does not hold the filter_chains, as it is the responsability of the
     * filter_chain manager.
     * It gets the filter_chains from the filter_chain manager.
     *
     * The visionServer is also the service point for listing service (media, exec,
     * filter_chain and filter)
     * and the start/stop execution.
     */
    class VisionServer : public sonia_common::ServiceServerManager<VisionServer> {
    public:
        using Ptr = std::shared_ptr<VisionServer>;

        explicit VisionServer(const ros::NodeHandle &nh);

        ~VisionServer() override;

    private:
        /**
         * Return a string of each execution separated by the current
         * COMPONENT_SEPARATOR given a list of the executions.
         */
        static std::string buildRosMessage(const std::vector<std::string> &name_vec);

        /**
         * When the vision client returns a filter name, it contains the index
         * of the filter in the filter_chain. This methods aims to extract
         * the index of the filter given it's full name.
         */
        static size_t extractFilterIndexFromUIName(const std::string &name);

        /**
         * Given a filter, this will return the name of the filter that the vision
         * client
         * is expecting (the name of the filter with its index of the filter in the
         * filter_chain).
         */
        static std::string constructFilterUIName(const std::string &name, const size_t &index);

        /**
         * \brief Copies a filter_chain which is not used by a running execution.
         *
         * Manages the ROS service sonia_common::CopyFilterchain.
         *
         * Here are the parameters of the service:
         *  * filter_chain_name_to_copy	The name of the filter_chain to copy.
         *  * filter_chain_new_name	The name of the new filter_chain (the copy).
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackCopyFilterChain(
                sonia_common::CopyFilterchain::Request &req,
                sonia_common::CopyFilterchain::Response &res
        );

        /**
         * \brief Gets the parameters for a filter.
         *
         * Manages the ROS service sonia_common::GetFilterchainFilterParam.
         *
         * Here are the parameters of the service:
         *  * filter_name Name of the filter contained in the filter_chain.
         *  * filter_chain_name Name of the filter_chain used by the execution.
         *  * execution_name Name of the execution which is running.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackGetFilterParam(
                sonia_common::GetFilterchainFilterParam::Request &req,
                sonia_common::GetFilterchainFilterParam::Response &res
        );

        /**
         * TODO Thibaut Mattio: Check this method, this is exacly the same
         * as the above one, is it really usefull ???
         *
         * \brief Gets the parameters for a filter.
         *
         * Manages the ROS service sonia_common::GetFilterchainFilterParam.
         *
         * Here are the parameters of the service:
         *  * filter_name Name of the filter contained in the filter_chain.
         *  * filter_chain_name Name of the filter_chain used by the execution.
         *  * execution_name Name of the execution which is running.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackGetFilterAllParam(
                sonia_common::GetFilterchainFilterAllParam::Request &req,
                sonia_common::GetFilterchainFilterAllParam::Response &res
        );

        /**
         * \brief Set the value of a parameter of a filter contained in a filter_chain
         * used by a running execution.
         *
         * Manages the ROS service sonia_common::SetFilterchainFilterParam.
         *
         * Here are the parameters of the service:
         *  * filter_chain_name Name of the filter_chain which contain the filter.
         *  * filter_name Name of the filter which contain the parameter.
         *  *	parameter_name Name of the parameter which the value has to be set.
         *  *	parameter_value The value of the parameter.
         *  *	execution_name Name of the running execution using the filter_chain.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackSetFilterParam(
                sonia_common::SetFilterchainFilterParam::Request &req,
                sonia_common::SetFilterchainFilterParam::Response &res
        );

        /**
         * \brief Gets the filters contained in a filter_chain.
         *
         * Manages the ROS service sonia_common::GetFilterchainFilterParam.
         *
         * Here are the parameters of the service:
         *  *	filter_chain_name Name of the filter_chain.
         *  *	execution_name Name of the execution. Let empty to get the filters list
         *                   associated with a filter_chain.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackGetFilter(
                sonia_common::GetFilterchainFilter::Request &req,
                sonia_common::GetFilterchainFilter::Response &res
        );

        /**
         * \brief Adds/Deletes a filter in a filter_chain used by a running execution.
         *
         * Manages the ROS service sonia_common::ManageFilterchainFilter.
         *
         * Here are the parameters of the service:
         *  * filter_chain_name	Name of the filter_chain.
         *  * commande The commande to proccess. 1 for ADD, 2 for DELETE.
         *  * filter_name Name of the filter.
         *  * execution_name Name of the execution.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackManageFilter(
                sonia_common::ManageFilterchainFilter::Request &req,
                sonia_common::ManageFilterchainFilter::Response &res
        );

        /**
         * \brief Creates/Deletes a filter_chain (the .fc fils in config directory).
         *
         * Manages the ROS service sonia_common::ManageFilterchain.
         *
         * Here are the parameters of the service:
         *  * filter_chain_name	Name of the filter_chain.
         *  * commande The commande to process. 1 for ADD, 2 for DELETE.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackManageFilterChain(
                sonia_common::ManageFilterchain::Request &req,
                sonia_common::ManageFilterchain::Response &res
        );

        /**
         * \brief Saves a filter_chain.
         *
         *
         * When a filter_chain is used by a running execution, the filters and their
         * parameters values can be modified.
         * The VisionServer doesn't store the new values until the user click to the
         * save button.
         * This will call this method, which send a request to the VisionServer
         * in order to save the filter_chain.
         * If the filter_chain is not used by a running execution, this call will
         * fail.
         *
         * Manages the ROS service sonia_common::SaveFilterchain.
         *
         * Here are the parameters of the service:
         *  * filter_chain_name Name of the filter_chain to save.
         *  * execution_name Name of the execution which use the filter_chain to
         *                   save.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackSaveFilterChain(
                sonia_common::SaveFilterchain::Request &req,
                sonia_common::SaveFilterchain::Response &res
        );

        /**
         * \brief Change the order of a filter in a filter_chain.
         *
         * Manages the ROS service sonia_common::SetFilterchainFilterOrder.
         *
         * Here are the parameters of the service:
         *  * execution_name Name of the execution which use the filter_chain.
         *  * filter_chain_name	Name of the filter_chain which contains the
         *                      filter to move.
         *  * filter_index Zero-based index of the filter.
         *  * commande 1=UP, 2=DOWN.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackSetFilterChainOrder(
                sonia_common::SetFilterchainFilterOrder::Request &req,
                sonia_common::SetFilterchainFilterOrder::Response &res
        );

        /**
         * \brief Get the filter_chain used by the running execution given as
         *parameter.
         *
         * Manages the ROS service sonia_common::GetFilterchainFromExecution.
         *
         * Here are the parameters of the service:
         *  * execution_name Name of the running execution.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackGetFilterChainExecution(
                sonia_common::GetFilterchainFromExecution::Request &req,
                sonia_common::GetFilterchainFromExecution::Response &res
        );

        /**
         * \brief Get the media used by the running execution given as parameter.
         *
         * Manages the ROS service sonia_common::GetMediaFromExecution.
         *
         * Here are the parameters of the service:
         *  * execution_name Name of the running execution.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackGetMediaExecution(
                sonia_common::GetMediaFromExecution::Request &req,
                sonia_common::GetMediaFromExecution::Response &res
        );

        /**
         * \brief Sets the observer to the filter given as parameter.
         *
         * As the processing of a filter_chain is like a pipe, it is possible to
         * observe the render of a specific filter and all the filters before.
         * This method set this "cursor".
         * The filter has to be used by a filter_chain used by a running execution.
         *
         * Manages the ROS service get_filter_chain_filter_param.
         *
         * Here are the parameters of the service:
         *  * execution_name Name of the running execution.
         *  * filter_chain_name Name of the filter_chain used by the execution.
         *  * filter_name Name of the filter used as the "cursor" of the observer.
         *
         * \param	req The ROS object containing the Request of the service.
         *             It contains all the parameters sent to the imageCallback
         * \param	res The ROS object containnig the Response of the service.
         *            It set values such as success for the service return state.
         * \return True if the imageCallback has succesfully processed the service call.
         */
        bool callbackSetObserver(
                sonia_common::SetFilterchainFilterObserver::Request &req,
                sonia_common::SetFilterchainFilterObserver::Response &res
        );

        bool callbackExecutionCmd(
                sonia_common::ExecuteCmd::Request &req,
                sonia_common::ExecuteCmd::Response &res
        );

        bool callbackInfoListCmd(
                sonia_common::GetInformationList::Request &req,
                sonia_common::GetInformationList::Response &res
        );

        /**
         * We must keep a reference to the initial node_handle for creating the
         * different topics and services outside the constructor.
         * This is mainly for performance purpose as we could also recreate a
         * ros::NodeHandle on the different instance of the objects.
         */
        ros::NodeHandle nh_;

        FilterChainManager filter_chain_mgr_;

        DetectionTaskManager detection_task_mgr_;
    };

    inline std::string VisionServer::buildRosMessage(const std::vector<std::string> &name_vec) {
        std::string msg;
        for (const auto &name : name_vec) {
            msg += name + ";";
        }
        return msg;
    }

    inline size_t VisionServer::extractFilterIndexFromUIName(const std::string &name) {
        size_t pos = name.find('_');
        if (pos == std::string::npos) {
            throw std::invalid_argument("The name of the filter could not be parsed");
        }
        std::string position = name.substr(pos + 1, name.size() - 1);
        return size_t(atoi(position.c_str()));
    }

    inline std::string VisionServer::constructFilterUIName(const std::string &name, const size_t &index) {
        return name + "_" + std::to_string(index);
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_SERVER_VISION_SERVER_H_
