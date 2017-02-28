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

#ifndef PROVIDER_VISION_SERVER_VISION_SERVER_H_
#define PROVIDER_VISION_SERVER_VISION_SERVER_H_

#include <proc_image_processing/copy_filterchain.h>
#include <proc_image_processing/execute_cmd.h>
#include <proc_image_processing/get_filterchain_filter.h>
#include <proc_image_processing/get_filterchain_filter_all_param.h>
#include <proc_image_processing/get_filterchain_filter_param.h>
#include <proc_image_processing/get_filterchain_from_execution.h>
#include <proc_image_processing/get_information_list.h>
#include <proc_image_processing/get_media_from_execution.h>
#include <proc_image_processing/manage_filterchain.h>
#include <proc_image_processing/manage_filterchain_filter.h>
#include <proc_image_processing/save_filterchain.h>
#include <proc_image_processing/set_filterchain_filter_observer.h>
#include <proc_image_processing/set_filterchain_filter_order.h>
#include <proc_image_processing/set_filterchain_filter_param.h>
#include <memory>
#include <string>
#include <vector>
#include "proc_image_processing/config.h"
#include "proc_image_processing/server/detection_task.h"
#include "proc_image_processing/server/detection_task_manager.h"
#include "proc_image_processing/server/filterchain_manager.h"

namespace proc_image_processing {

using namespace proc_image_processing;
/**
 * Vision server is the main class of the system
 * It's job is to assemble and connect the pieces to create execution
 * It is the owner of the active DetectionTask and MediaStreamer
 * It does not hold the filterchains, as it is the responsability of the
 * filterchain manager.
 * It gets the filterchains from the filterchain manager.
 *
 * The visionServer is also the service point for listing service (media, exec,
 * filterchain and filter)
 * and the start/stop execution.
 */
class VisionServer : public atlas::ServiceServerManager<VisionServer> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<VisionServer>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit VisionServer(const ros::NodeHandle &nh);

  ~VisionServer();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Return a string of each execution separated by the current
   * COMPONENT_SEPARATOR given a list of the executions.
   */
  std::string BuildRosMessage(const std::vector<std::string> &name_vec) const;

  /**
   * When the vision client returns a filter name, it contains the index
   * of the filter in the filterchain. This methods aims to extract
   * the index of the filter given it's full name.
   */
  size_t ExtractFilterIndexFromUIName(const std::string &name) const;

  /**
   * Given a filter, this will return the name of the filter that the vision
   * client
   * is expecting (the name of the filter with its index of the filter in the
   * filterchain).
   */
  std::string ConstructFilterUIName(const std::string &name,
                                    const size_t &index) const;

  /**
   * \brief Copies a filterchain which is not used by a running execution.
   *
   * Manages the ROS service copy_filterchain.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name_to_copy	The name of the filterchain to copy.
   *  * filter_chain_new_name	The name of the new filterchain (the copy).
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackCopyFc(copy_filterchain::Request &rqst,
                      copy_filterchain::Response &rep);

  /**
   * \brief Gets the parameters for a filter.
   *
   * Manages the ROS service get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * filter_name Name of the filter contained in the filterchain.
   *  * filter_chain_name Name of the filterchain used by the execution.
   *  * execution_name Name of the execution which is running.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFilterParam(get_filterchain_filter_param::Request &rqst,
                              get_filterchain_filter_param::Response &rep);

  /**
   * TODO Thibaut Mattio: Check this method, this is exacly the same
   * as the above one, is it really usefull ???
   *
   * \brief Gets the parameters for a filter.
   *
   * Manages the ROS service get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * filter_name Name of the filter contained in the filterchain.
   *  * filter_chain_name Name of the filterchain used by the execution.
   *  * execution_name Name of the execution which is running.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFilterAllParam(
      get_filterchain_filter_all_param::Request &rqst,
      get_filterchain_filter_all_param::Response &rep);

  /**
   * \brief Set the value of a parameter of a filter contained in a filterchain
   * used by a running execution.
   *
   * Manages the ROS service set_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name Name of the filterchain which contain the filter.
   *  * filter_name Name of the filter which contain the parameter.
   *  *	parameter_name Name of the parameter which the value has to be set.
   *  *	parameter_value The value of the parameter.
   *  *	execution_name Name of the running execution using the filterchain.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSetFilterParam(set_filterchain_filter_param::Request &rqst,
                              set_filterchain_filter_param::Response &rep);

  /**
   * \brief Gets the filters contained in a filterchain.
   *
   * Manages the ROS service get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  *	filter_chain_name Name of the filterchain.
   *  *	execution_name Name of the execution. Let empty to get the filters list
   *                   associated with a filterchain.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFilter(get_filterchain_filter::Request &rqst,
                         get_filterchain_filter::Response &rep);

  /**
   * \brief Adds/Deletes a filter in a filterchain used by a running execution.
   *
   * Manages the ROS service manage_filterchain_filter.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name	Name of the filterchain.
   *  * commande The commande to proccess. 1 for ADD, 2 for DELETE.
   *  * filter_name Name of the filter.
   *  * execution_name Name of the execution.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackManageFilter(manage_filterchain_filter::Request &rqst,
                            manage_filterchain_filter::Response &rep);

  /**
   * \brief Creates/Deletes a filterchain (the .fc fils in config directory).
   *
   * Manages the ROS service manage_filterchain.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name	Name of the filterchain.
   *  * commande The commande to process. 1 for ADD, 2 for DELETE.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackManageFc(manage_filterchain::Request &rqst,
                        manage_filterchain::Response &rep);

  /**
   * \brief Saves a filterchain.
   *
   *
   * When a filterchain is used by a running execution, the filters and their
   * parameters values can be modified.
   * The VisionServer doesn't store the new values until the user click to the
   * save button.
   * This will call this method, which send a request to the VisionServer
   * in order to save the filterchain.
   * If the filterchain is not used by a running execution, this call will
   * fail.
   *
   * Manages the ROS service save_filterchain.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name Name of the filterchain to save.
   *  * execution_name Name of the execution which use the filterchain to
   *                   save.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSaveFc(save_filterchain::Request &rqst,
                      save_filterchain::Response &rep);

  /**
   * \brief Change the order of a filter in a filterchain.
   *
   * Manages the ROS service set_filterchain_filter_order.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the execution which use the filterchain.
   *  * filter_chain_name	Name of the filterchain which contains the
   *                      filter to move.
   *  * filter_index Zero-based index of the filter.
   *  * commande 1=UP, 2=DOWN.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSetFcOrder(set_filterchain_filter_order::Request &rqst,
                          set_filterchain_filter_order::Response &rep);

  /**
   * \brief Get the filterchain used by the running execution given as
   *parameter.
   *
   * Manages the ROS service get_filterchain_from_execution.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the running execution.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFcFromExec(get_filterchain_from_execution::Request &rqst,
                             get_filterchain_from_execution::Response &rep);

  /**
   * \brief Get the media used by the running execution given as parameter.
   *
   * Manages the ROS service get_media_from_execution.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the running execution.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetMediaFromExec(get_media_from_execution::Request &rqst,
                                get_media_from_execution::Response &rep);

  /**
   * \brief Sets the observer to the filter given as parameter.
   *
   * As the processing of a filterchain is like a pipe, it is possible to
   * observe the render of a specific filter and all the filters before.
   * This method set this "cursor".
   * The filter has to be used by a filterchain used by a running execution.
   *
   * Manages the ROS service get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the running execution.
   *  * filter_chain_name Name of the filterchain used by the execution.
   *  * filter_name Name of the filter used as the "cursor" of the observer.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSetObserver(set_filterchain_filter_observer::Request &rqst,
                           set_filterchain_filter_observer::Response &rep);

  bool CallbackExecutionCMD(proc_image_processing::execute_cmd::Request &rqst,
                            proc_image_processing::execute_cmd::Response &rep);

  bool CallbackInfoListCMD(
      proc_image_processing::get_information_list::Request &rqst,
      proc_image_processing::get_information_list::Response &rep);


  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * We must keep a reference to the initial node_handle for creating the
   * different topics and services outside the constructor.
   * This is mainly for performance purpose as we could also recreate a
   * ros::NodeHandle on the different instance of the objects.
   */
  ros::NodeHandle nh_;

  FilterchainManager filterchain_mgr_;

  DetectionTaskManager detection_task_mgr_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline std::string VisionServer::BuildRosMessage(
    const std::vector<std::string> &name_vec) const {
  std::string msg("");
  for (const auto &name : name_vec) {
    msg += name + ";";
  }
  return msg;
}

//------------------------------------------------------------------------------
//
inline size_t VisionServer::ExtractFilterIndexFromUIName(
    const std::string &name) const {
  size_t pos = name.find("_");
  if (pos == std::string::npos) {
    throw std::invalid_argument("The name of the filter could not be parsed");
  }
  std::string position = name.substr(pos + 1, name.size() - 1);
  return size_t(atoi(position.c_str()));
}

//------------------------------------------------------------------------------
//
inline std::string VisionServer::ConstructFilterUIName(
    const std::string &name, const size_t &index) const {
  return name + "_" + std::to_string(index);
}

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_SERVER_VISION_SERVER_H_
