/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include <fstream>
#include "proc_image_processing/server/vision_server.h"
#include <sonia_common/ChangeNetwork.h>


namespace proc_image_processing {

  //==============================================================================
  // C / D T O R S   S E C T I O N

  //------------------------------------------------------------------------------
  //
  VisionServer::VisionServer(const ros::NodeHandle& nh)
    : sonia_common::ServiceServerManager<VisionServer>(),
    nh_(nh),
    filterchain_mgr_() {
    auto base_node_name = std::string{ kRosNodeName };

    RegisterService<sonia_common::ExecuteCmd>(base_node_name + "execute_cmd",
      &VisionServer::CallbackExecutionCMD, *this);

    RegisterService<sonia_common::GetInformationList>(base_node_name + "get_information_list",
      &VisionServer::CallbackInfoListCMD,
      *this);

    RegisterService<sonia_common::CopyFilterchain>(base_node_name + "copy_filterchain",
      &VisionServer::CallbackCopyFc, *this);

    RegisterService<sonia_common::GetFilterchainFilterAllParam>(
      base_node_name + "get_filterchain_filter_all_param",
      &VisionServer::CallbackGetFilterAllParam, *this);

    RegisterService<sonia_common::GetFilterchainFilterParam>(
      base_node_name + "get_filterchain_filter_param",
      &VisionServer::CallbackGetFilterParam, *this);

    RegisterService<sonia_common::SetFilterchainFilterParam>(
      base_node_name + "set_filterchain_filter_param",
      &VisionServer::CallbackSetFilterParam, *this);

    RegisterService<sonia_common::GetFilterchainFilter>(
      base_node_name + "get_filterchain_filter",
      &VisionServer::CallbackGetFilter, *this);

    RegisterService<sonia_common::ManageFilterchainFilter>(
      base_node_name + "manage_filterchain_filter",
      &VisionServer::CallbackManageFilter, *this);

    RegisterService<sonia_common::ManageFilterchain>(base_node_name + "manage_filterchain",
      &VisionServer::CallbackManageFc, *this);

    RegisterService<sonia_common::SaveFilterchain>(base_node_name + "save_filterchain",
      &VisionServer::CallbackSaveFc, *this);

    RegisterService<sonia_common::SetFilterchainFilterOrder>(
      base_node_name + "set_filterchain_filter_order",
      &VisionServer::CallbackSetFcOrder, *this);

    RegisterService<sonia_common::GetFilterchainFromExecution>(
      base_node_name + "get_filterchain_from_execution",
      &VisionServer::CallbackGetFcFromExec, *this);

    RegisterService<sonia_common::GetMediaFromExecution>(
      base_node_name + "get_media_from_execution",
      &VisionServer::CallbackGetMediaFromExec, *this);

    RegisterService<sonia_common::SetFilterchainFilterObserver>(
      base_node_name + "set_filterchain_filter_observer",
      &VisionServer::CallbackSetObserver, *this);

    deep_network_service = ros::NodeHandle("~").serviceClient<sonia_common::ChangeNetwork>("/deep_detector/change_network");
  }

  //------------------------------------------------------------------------------
  //
  VisionServer::~VisionServer() {}

  //==============================================================================
  // M E T H O D   S E C T I O N

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackExecutionCMD(sonia_common::ExecuteCmd::Request& rqst,
    sonia_common::ExecuteCmd::Response& rep) {
    if (rqst.cmd == rqst.START) {
      try {
        ROS_INFO("--- Starting Execution ---");
        ROS_INFO("Node: %s, Filterchain: %s, Media: %s", rqst.node_name.c_str(),
          rqst.filterchain_name.c_str(), rqst.media_name.c_str());
        Filterchain::Ptr filterchain =
          filterchain_mgr_.InstanciateFilterchain(rqst.filterchain_name);

        rep.response = detection_task_mgr_.StartDetectionTask(rqst.media_name, filterchain,
          rqst.node_name);
        sonia_common::ChangeNetwork network;
        network.request.task = rqst.filterchain_name;
        deep_network_service.call(network);
        ROS_INFO("Starting topic: %s", rep.response.c_str());
      }
      catch (const std::exception& e) {
        ROS_ERROR("Starting execution error: %s", e.what());
        rep.response = "";
        // WHAAATTT IT FAILED BUT YOU RETURN TRUE!?!?!?
        // If we return false, ROS consider the call failed and do not update the response field,
        // so if you reuse the same variable (like in black_box_test.cc, it does not get updated.
        // Here the call did not fail, we simply deny the service.
        return true;
      }
    }
    else if (rqst.cmd == rqst.STOP) {
      try {
        ROS_INFO(" ");
        ROS_INFO("--- Stoping Execution ---");
        ROS_INFO("Node: %s, Filterchain: %s, Media: %s", rqst.node_name.c_str(),
          rqst.filterchain_name.c_str(), rqst.media_name.c_str());

        auto fc =
          detection_task_mgr_.GetFilterchainFromDetectionTask(rqst.node_name);
        if (fc == nullptr) {
          ROS_ERROR("Filterchain does not exist, cannot close execution.");
          return false;
        }

        detection_task_mgr_.StopDetectionTask(rqst.node_name);

        filterchain_mgr_.StopFilterchain(fc);

      }
      catch (const std::exception& e) {
        ROS_ERROR("Closing execution error: %s", e.what());
        return false;
      }
    }
    return true;
  }

  //------------------------------------------------------------------------------
  //
  bool proc_image_processing::VisionServer::CallbackInfoListCMD(
    sonia_common::GetInformationList::Request& rqst, sonia_common::GetInformationList::Response& rep) {
    if (rqst.cmd == rqst.EXEC) {
      rep.list = BuildRosMessage(detection_task_mgr_.GetAllDetectionTasksName());
    }
    else if (rqst.cmd == rqst.FILTERCHAIN) {
      rep.list = BuildRosMessage(filterchain_mgr_.GetAllFilterchainName());
    }
    else if (rqst.cmd == rqst.FILTERS) {
      rep.list = proc_image_processing::FilterFactory::GetFilterList();
    }
    else if (rqst.cmd == rqst.MEDIA) {
      rep.list = BuildRosMessage(detection_task_mgr_.GetAllMediasName());
    }

    return true;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackCopyFc(sonia_common::CopyFilterchain::Request& rqst,
    sonia_common::CopyFilterchain::Response& rep) {
    std::ifstream src(
      filterchain_mgr_.GetFilterchainPath(rqst.filterchain_to_copy).c_str(),
      std::ios::binary);

    std::ofstream dst(
      filterchain_mgr_.GetFilterchainPath(rqst.filterchain_new_name).c_str(),
      std::ios::binary);
    dst << src.rdbuf();
    rep.success = true;
    return rep.success;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackGetFilterParam(
    sonia_common::GetFilterchainFilterParam::Request& rqst,
    sonia_common::GetFilterchainFilterParam::Response& rep) {
    rep.list = "";

    std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
    Filterchain::Ptr filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(execution_name);

    if (filterchain != nullptr) {
      rep.list = filterchain->GetFilterParameterValue(
        ExtractFilterIndexFromUIName(rqst.filter), rqst.parameter);
      return true;
    }
    ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
    return false;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackGetFilterAllParam(
    sonia_common::GetFilterchainFilterAllParam::Request& rqst,
    sonia_common::GetFilterchainFilterAllParam::Response& rep) {
    rep.list = "";

    std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
    Filterchain::Ptr filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(execution_name);

    if (filterchain != nullptr) {
      auto parameters = filterchain->GetFilterAllParameters(
        ExtractFilterIndexFromUIName(rqst.filter));
      std::vector<std::string> parameter_names;
      for (const auto& parameter : parameters) {
        parameter_names.push_back(parameter->ToString());
      }
      rep.list = BuildRosMessage(parameter_names);
      return true;
    }
    ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
    return false;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackSetFilterParam(
    sonia_common::SetFilterchainFilterParam::Request& rqst,
    sonia_common::SetFilterchainFilterParam::Response& rep) {
    rep.success = rep.FAIL;

    std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
    Filterchain::Ptr filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(execution_name);

    if (filterchain != nullptr) {
      filterchain->SetFilterParameterValue(
        ExtractFilterIndexFromUIName(rqst.filter), rqst.parameter, rqst.value);
      rep.success = rep.SUCCESS;
      return true;
    }
    ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
    return false;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackGetFilter(sonia_common::GetFilterchainFilter::Request& rqst,
    sonia_common::GetFilterchainFilter::Response& rep) {
    rep.list = "";

    std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
    Filterchain::Ptr filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(execution_name);

    if (filterchain != nullptr) {
      auto filters = filterchain->GetAllFilters();
      std::vector<std::string> filter_names;
      for (size_t i = 0; i < filters.size(); ++i) {
        filter_names.push_back(
          ConstructFilterUIName(filters.at(i)->GetName(), i));
      }
      rep.list = BuildRosMessage(filter_names);
      return true;
    }

    std::string log_txt = "DetectionTask " + execution_name +
      " does not exist or does not use this filterchain: " +
      filterchain_name + " on get filter's param request.";
    ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
    return false;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackSetObserver(
    sonia_common::SetFilterchainFilterObserver::Request& rqst,
    sonia_common::SetFilterchainFilterObserver::Response& rep) {
    // For now ignoring filterchain name, but when we will have multiple,
    // we will have to check the name and find the good filterchain
    Filterchain::Ptr filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(rqst.execution);

    if (filterchain != nullptr) {
      rep.result = rep.SUCCESS;
      filterchain->SetObserver(ExtractFilterIndexFromUIName(rqst.filter));
      return true;
    }

    rep.result = rep.FAIL;
    ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filters request",
      rqst.execution.c_str(), rqst.filterchain.c_str());
    return false;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackManageFilter(
    sonia_common::ManageFilterchainFilter::Request& rqst,
    sonia_common::ManageFilterchainFilter::Response& rep) {
    const auto& filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(rqst.exec_name);
    rep.success = 1;
    if (filterchain != nullptr) {
      if (rqst.cmd == rqst.ADD) {
        filterchain->AddFilter(rqst.filter);
      }
      else if (rqst.cmd == rqst.DELETE) {
        filterchain->RemoveFilter(ExtractFilterIndexFromUIName(rqst.filter));
      }
    }
    else {
      rep.success = 0;
    }

    return rep.success;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackManageFc(sonia_common::ManageFilterchain::Request& rqst,
    sonia_common::ManageFilterchain::Response& rep) {
    std::string filterchain_name(rqst.filterchain);
    bool response = true;
    if (rqst.cmd == rqst.ADD) {
      filterchain_mgr_.CreateFilterchain(filterchain_name);
      rep.success = rep.SUCCESS;
    }
    else if (rqst.cmd == rqst.DELETE) {
      filterchain_mgr_.EraseFilterchain(filterchain_name);
    }
    return response;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackSaveFc(sonia_common::SaveFilterchain::Request& rqst,
    sonia_common::SaveFilterchain::Response& rep) {
    std::string execution_name(rqst.exec_name);
    std::string filterchain_name(rqst.filterchain);
    if (rqst.cmd == rqst.SAVE) {
      detection_task_mgr_.GetFilterchainFromDetectionTask(rqst.exec_name)
        ->Serialize();
      rep.success = rep.SUCCESS;
    }
    return true;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackSetFcOrder(
    sonia_common::SetFilterchainFilterOrder::Request& rqst,
    sonia_common::SetFilterchainFilterOrder::Response& rep) {
    ROS_INFO("Call to set_filterchain_filter_order.");

    rep.success = rep.SUCCESS;
    auto filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(rqst.exec_name);
    if (rqst.cmd == rqst.UP) {
      filterchain->MoveFilterUp(rqst.filter_index);
    }
    else if (rqst.cmd == rqst.DOWN) {
      filterchain->MoveFilterDown(rqst.filter_index);
    }
    else {
      ROS_INFO("Filter index provided was invalid");
      rep.success = rep.FAIL;
    }

    return rep.success;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackGetFcFromExec(
    sonia_common::GetFilterchainFromExecution::Request& rqst,
    sonia_common::GetFilterchainFromExecution::Response& rep) {
    ROS_INFO("Call to get_filterchain_from_execution.");
    std::string execution_name(rqst.exec_name);
    Filterchain::Ptr filterchain =
      detection_task_mgr_.GetFilterchainFromDetectionTask(execution_name);

    if (filterchain != nullptr) {
      rep.list = filterchain->GetName();
      return true;
    }
    ROS_INFO(
      "DetectionTask %s does not exist or does not use this filterchain "
      "on get filters request.",
      execution_name.c_str());
    return false;
  }

  //------------------------------------------------------------------------------
  //
  bool VisionServer::CallbackGetMediaFromExec(
    sonia_common::GetMediaFromExecution::Request& rqst,
    sonia_common::GetMediaFromExecution::Response& rep) {
    ROS_INFO("Call to get_media_from_execution.");
    auto response = "media_" + rqst.exec_name;
    rep.list = response;
    return true;
  }

}  // namespace proc_image_processing
