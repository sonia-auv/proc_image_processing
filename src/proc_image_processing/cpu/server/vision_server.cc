#include <fstream>
#include <proc_image_processing/cpu/server/vision_server.h>
#include <sonia_common/ChangeNetwork.h>

namespace proc_image_processing {

    VisionServer::VisionServer(const ros::NodeHandle &nh)
            : sonia_common::ServiceServerManager<VisionServer>(),
              nh_(nh),
              filter_chain_mgr_() {
        auto base_node_name = std::string{kRosNodeName};

        RegisterService<sonia_common::ExecuteCmd>(base_node_name + "/execute_cmd",
                                                  &VisionServer::callbackExecutionCmd, *this);

        RegisterService<sonia_common::GetInformationList>(base_node_name + "/get_information_list",
                                                          &VisionServer::callbackInfoListCmd,
                                                          *this);

        RegisterService<sonia_common::CopyFilterchain>(base_node_name + "/copy_filterchain",
                                                       &VisionServer::callbackCopyFilterChain, *this);

        RegisterService<sonia_common::GetFilterchainFilterAllParam>(
                base_node_name + "/get_filterchain_filter_all_param",
                &VisionServer::callbackGetFilterAllParam, *this);

        RegisterService<sonia_common::GetFilterchainFilterParam>(
                base_node_name + "/get_filterchain_filter_param",
                &VisionServer::callbackGetFilterParam, *this);

        RegisterService<sonia_common::SetFilterchainFilterParam>(
                base_node_name + "/set_filterchain_filter_param",
                &VisionServer::callbackSetFilterParam, *this);

        RegisterService<sonia_common::GetFilterchainFilter>(
                base_node_name + "/get_filterchain_filter",
                &VisionServer::callbackGetFilter, *this);

        RegisterService<sonia_common::ManageFilterchainFilter>(
                base_node_name + "/manage_filterchain_filter",
                &VisionServer::callbackManageFilter, *this);

        RegisterService<sonia_common::ManageFilterchain>(base_node_name + "/manage_filterchain",
                                                         &VisionServer::callbackManageFilterChain, *this);

        RegisterService<sonia_common::SaveFilterchain>(base_node_name + "/save_filterchain",
                                                       &VisionServer::callbackSaveFilterChain, *this);

        RegisterService<sonia_common::SetFilterchainFilterOrder>(
                base_node_name + "/set_filterchain_filter_order",
                &VisionServer::callbackSetFilterChainOrder, *this);

        RegisterService<sonia_common::GetFilterchainFromExecution>(
                base_node_name + "/get_filterchain_from_execution",
                &VisionServer::callbackGetFilterChainExecution, *this);

        RegisterService<sonia_common::GetMediaFromExecution>(
                base_node_name + "/get_media_from_execution",
                &VisionServer::callbackGetMediaExecution, *this);

        RegisterService<sonia_common::SetFilterchainFilterObserver>(
                base_node_name + "/set_filterchain_filter_observer",
                &VisionServer::callbackSetObserver, *this);

        deep_network_service = ros::NodeHandle("~").serviceClient<sonia_common::ChangeNetwork>(
                "/deep_detector/change_network");
    }

    VisionServer::~VisionServer() {}

    bool VisionServer::callbackExecutionCmd(sonia_common::ExecuteCmd::Request &rqst,
                                            sonia_common::ExecuteCmd::Response &rep) {
        if (rqst.cmd == rqst.START) {
            try {
                ROS_INFO("--- Starting Execution ---");
                ROS_INFO("Node: %s, FilterChain: %s, Media: %s", rqst.node_name.c_str(),
                         rqst.filterchain_name.c_str(), rqst.media_name.c_str());
                FilterChain::Ptr filterchain =
                        filter_chain_mgr_.createFilterChain(rqst.filterchain_name);

                rep.response = detection_task_mgr_.StartDetectionTask(rqst.media_name, filterchain,
                                                                      rqst.node_name);
                sonia_common::ChangeNetwork network;
                network.request.task = rqst.filterchain_name;
                deep_network_service.call(network);
                ROS_INFO("Starting topic: %s", rep.response.c_str());
            }
            catch (const std::exception &e) {
                ROS_ERROR("Starting execution error: %s", e.what());
                rep.response = "";
                // WHAAATTT IT FAILED BUT YOU RETURN TRUE!?!?!?
                // If we return false, ROS consider the call failed and do not update the response field,
                // so if you reuse the same variable (like in black_box_test.cc, it does not get updated.
                // Here the call did not fail, we simply deny the service.
                return true;
            }
        } else if (rqst.cmd == rqst.STOP) {
            try {
                ROS_INFO(" ");
                ROS_INFO("--- Stoping Execution ---");
                ROS_INFO("Node: %s, FilterChain: %s, Media: %s", rqst.node_name.c_str(),
                         rqst.filterchain_name.c_str(), rqst.media_name.c_str());

                auto fc =
                        detection_task_mgr_.getFilterChainFromDetectionTask(rqst.node_name);
                if (fc == nullptr) {
                    ROS_ERROR("FilterChain does not exist, cannot close execution.");
                    return false;
                }

                detection_task_mgr_.stopDetectionTask(rqst.node_name);

                filter_chain_mgr_.stopFilterChain(fc);

            }
            catch (const std::exception &e) {
                ROS_ERROR("Closing execution error: %s", e.what());
                return false;
            }
        }
        return true;
    }

    bool VisionServer::callbackInfoListCmd(
            sonia_common::GetInformationList::Request &rqst, sonia_common::GetInformationList::Response &rep) {
        if (rqst.cmd == rqst.EXEC) {
            rep.list = buildRosMessage(detection_task_mgr_.getDetectionTasksNames());
        } else if (rqst.cmd == rqst.FILTERCHAIN) {
            rep.list = buildRosMessage(filter_chain_mgr_.getFilterChainsNames());
        } else if (rqst.cmd == rqst.FILTERS) {
            rep.list = FilterFactory::getFilters();
        } else if (rqst.cmd == rqst.MEDIA) {
            rep.list = buildRosMessage(detection_task_mgr_.getMediasNames());
        }

        return true;
    }

    bool VisionServer::callbackCopyFilterChain(sonia_common::CopyFilterchain::Request &rqst,
                                               sonia_common::CopyFilterchain::Response &rep) {
        std::ifstream src(
                filter_chain_mgr_.getFilterChainPath(rqst.filterchain_to_copy).c_str(),
                std::ios::binary);

        std::ofstream dst(
                filter_chain_mgr_.getFilterChainPath(rqst.filterchain_new_name).c_str(),
                std::ios::binary);
        dst << src.rdbuf();
        rep.success = true;
        return rep.success;
    }

    bool VisionServer::callbackGetFilterParam(
            sonia_common::GetFilterchainFilterParam::Request &rqst,
            sonia_common::GetFilterchainFilterParam::Response &rep) {
        rep.list = "";

        std::string execution_name(rqst.exec_name),
                filterchain_name(rqst.filterchain);
        FilterChain::Ptr filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filterchain != nullptr) {
            rep.list = filterchain->getFilterParameterValue(
                    extractFilterIndexFromUIName(rqst.filter), rqst.parameter);
            return true;
        }
        ROS_INFO(
                "DetectionTask %s does not exist or does not use this "
                "filterchain: %s on get filter's param request.",
                execution_name.c_str(), filterchain_name.c_str());
        return false;
    }

    bool VisionServer::callbackGetFilterAllParam(
            sonia_common::GetFilterchainFilterAllParam::Request &rqst,
            sonia_common::GetFilterchainFilterAllParam::Response &rep) {
        rep.list = "";

        std::string execution_name(rqst.exec_name),
                filterchain_name(rqst.filterchain);
        FilterChain::Ptr filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filterchain != nullptr) {
            auto parameters = filterchain->getFilterParameters(
                    extractFilterIndexFromUIName(rqst.filter));
            std::vector<std::string> parameter_names;
            for (const auto &parameter : parameters) {
                parameter_names.push_back(parameter->toString());
            }
            rep.list = buildRosMessage(parameter_names);
            return true;
        }
        ROS_INFO(
                "DetectionTask %s does not exist or does not use this "
                "filterchain: %s on get filter's param request.",
                execution_name.c_str(), filterchain_name.c_str());
        return false;
    }

    bool VisionServer::callbackSetFilterParam(
            sonia_common::SetFilterchainFilterParam::Request &rqst,
            sonia_common::SetFilterchainFilterParam::Response &rep) {
        rep.success = rep.FAIL;

        std::string execution_name(rqst.exec_name),
                filterchain_name(rqst.filterchain);
        FilterChain::Ptr filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filterchain != nullptr) {
            filterchain->setFilterParameterValue(
                    extractFilterIndexFromUIName(rqst.filter), rqst.parameter, rqst.value);
            rep.success = rep.SUCCESS;
            return true;
        }
        ROS_INFO(
                "DetectionTask %s does not exist or does not use this "
                "filterchain: %s on get filter's param request.",
                execution_name.c_str(), filterchain_name.c_str());
        return false;
    }

    bool VisionServer::callbackGetFilter(sonia_common::GetFilterchainFilter::Request &rqst,
                                         sonia_common::GetFilterchainFilter::Response &rep) {
        rep.list = "";

        std::string execution_name(rqst.exec_name),
                filterchain_name(rqst.filterchain);
        FilterChain::Ptr filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filterchain != nullptr) {
            auto filters = filterchain->getFilters();
            std::vector<std::string> filter_names;
            for (size_t i = 0; i < filters.size(); ++i) {
                filter_names.push_back(
                        constructFilterUIName(filters.at(i)->getName(), i));
            }
            rep.list = buildRosMessage(filter_names);
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

    bool VisionServer::callbackSetObserver(
            sonia_common::SetFilterchainFilterObserver::Request &rqst,
            sonia_common::SetFilterchainFilterObserver::Response &rep) {
        // For now ignoring filterchain name, but when we will have multiple,
        // we will have to check the name and find the good filterchain
        FilterChain::Ptr filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(rqst.execution);

        if (filterchain != nullptr) {
            rep.result = rep.SUCCESS;
            filterchain->setObserver(extractFilterIndexFromUIName(rqst.filter));
            return true;
        }

        rep.result = rep.FAIL;
        ROS_INFO(
                "DetectionTask %s does not exist or does not use this "
                "filterchain: %s on get filters request",
                rqst.execution.c_str(), rqst.filterchain.c_str());
        return false;
    }

    bool VisionServer::callbackManageFilter(
            sonia_common::ManageFilterchainFilter::Request &rqst,
            sonia_common::ManageFilterchainFilter::Response &rep) {
        const auto &filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(rqst.exec_name);
        rep.success = 1;
        if (filterchain != nullptr) {
            if (rqst.cmd == rqst.ADD) {
                filterchain->addFilter(rqst.filter);
            } else if (rqst.cmd == rqst.DELETE) {
                filterchain->removeFilter(extractFilterIndexFromUIName(rqst.filter));
            }
        } else {
            rep.success = 0;
        }

        return rep.success;
    }

    bool VisionServer::callbackManageFilterChain(sonia_common::ManageFilterchain::Request &rqst,
                                                 sonia_common::ManageFilterchain::Response &rep) {
        std::string filterchain_name(rqst.filterchain);
        bool response = true;
        if (rqst.cmd == rqst.ADD) {
            filter_chain_mgr_.addFilterChain(filterchain_name);
            rep.success = rep.SUCCESS;
        } else if (rqst.cmd == rqst.DELETE) {
            filter_chain_mgr_.deleteFilterChain(filterchain_name);
        }
        return response;
    }

    bool VisionServer::callbackSaveFilterChain(sonia_common::SaveFilterchain::Request &rqst,
                                               sonia_common::SaveFilterchain::Response &rep) {
        std::string execution_name(rqst.exec_name);
        std::string filterchain_name(rqst.filterchain);
        if (rqst.cmd == rqst.SAVE) {
            detection_task_mgr_.getFilterChainFromDetectionTask(rqst.exec_name)
                    ->serialize();
            rep.success = rep.SUCCESS;
        }
        return true;
    }

    bool VisionServer::callbackSetFilterChainOrder(
            sonia_common::SetFilterchainFilterOrder::Request &rqst,
            sonia_common::SetFilterchainFilterOrder::Response &rep) {
        ROS_INFO("Call to set_filterchain_filter_order.");

        rep.success = rep.SUCCESS;
        auto filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(rqst.exec_name);
        if (rqst.cmd == rqst.UP) {
            filterchain->moveFilterUp(rqst.filter_index);
        } else if (rqst.cmd == rqst.DOWN) {
            filterchain->moveFilterDown(rqst.filter_index);
        } else {
            ROS_INFO("Filter index provided was invalid");
            rep.success = rep.FAIL;
        }

        return rep.success;
    }

    bool VisionServer::callbackGetFilterChainExecution(
            sonia_common::GetFilterchainFromExecution::Request &rqst,
            sonia_common::GetFilterchainFromExecution::Response &rep) {
        ROS_INFO("Call to get_filterchain_from_execution.");
        std::string execution_name(rqst.exec_name);
        FilterChain::Ptr filterchain =
                detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filterchain != nullptr) {
            rep.list = filterchain->getName();
            return true;
        }
        ROS_INFO(
                "DetectionTask %s does not exist or does not use this filterchain "
                "on get filters request.",
                execution_name.c_str());
        return false;
    }

    bool VisionServer::callbackGetMediaExecution(
            sonia_common::GetMediaFromExecution::Request &rqst,
            sonia_common::GetMediaFromExecution::Response &rep) {
        ROS_INFO("Call to get_media_from_execution.");
        auto response = "media_" + rqst.exec_name;
        rep.list = response;
        return true;
    }

}  // namespace proc_image_processing
