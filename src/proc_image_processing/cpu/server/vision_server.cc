#include <fstream>
#include <proc_image_processing/cpu/server/vision_server.h>
#include <sonia_common/ChangeNetwork.h>

namespace proc_image_processing {

    VisionServer::VisionServer(const ros::NodeHandle &nh)
            : sonia_common::ServiceServerManager<VisionServer>(), nh_(nh) {
        auto base_node_name = std::string{kRosNodeName};

        RegisterService<sonia_common::ExecuteCmd>(
                base_node_name + "/execute_cmd",
                &VisionServer::callbackExecutionCmd,
                *this
        );

        RegisterService<sonia_common::GetInformationList>(
                base_node_name + "/get_information_list",
                &VisionServer::callbackInfoListCmd,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::CopyFilterchain>(
                base_node_name + "/copy_filterchain",
                &VisionServer::callbackCopyFilterChain,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::GetFilterchainFilterAllParam>(
                base_node_name + "/get_filterchain_filter_all_param",
                &VisionServer::callbackGetFilterAllParam,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::GetFilterchainFilterParam>(
                base_node_name + "/get_filterchain_filter_param",
                &VisionServer::callbackGetFilterParam,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::SetFilterchainFilterParam>(
                base_node_name + "/set_filterchain_filter_param",
                &VisionServer::callbackSetFilterParam,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::GetFilterchainFilter>(
                base_node_name + "/get_filterchain_filter",
                &VisionServer::callbackGetFilter,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::ManageFilterchainFilter>(
                base_node_name + "/manage_filterchain_filter",
                &VisionServer::callbackManageFilter,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::ManageFilterchain>(
                base_node_name + "/manage_filterchain",
                &VisionServer::callbackManageFilterChain,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::SaveFilterchain>(
                base_node_name + "/save_filterchain",
                &VisionServer::callbackSaveFilterChain,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::SetFilterchainFilterOrder>(
                base_node_name + "/set_filterchain_filter_order",
                &VisionServer::callbackSetFilterChainOrder,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::GetFilterchainFromExecution>(
                base_node_name + "/get_filterchain_from_execution",
                &VisionServer::callbackGetFilterChainExecution,
                *this
        );

        RegisterService<sonia_common::GetMediaFromExecution>(
                base_node_name + "/get_media_from_execution",
                &VisionServer::callbackGetMediaExecution,
                *this
        );

        // TODO rename filterchain to filter_chain (impacts octopus-telemetry)
        RegisterService<sonia_common::SetFilterchainFilterObserver>(
                base_node_name + "/set_filterchain_filter_observer",
                &VisionServer::callbackSetObserver,
                *this
        );
    }

    VisionServer::~VisionServer() = default;

    bool VisionServer::callbackExecutionCmd(
            sonia_common::ExecuteCmd::Request &req,
            sonia_common::ExecuteCmd::Response &res
    ) {
        if (req.cmd == req.START) {
            try {
                ROS_INFO("--- Starting Execution ---");
                ROS_INFO(
                        "Node: %s, FilterChain: %s, Media: %s",
                        req.node_name.c_str(),
                        req.filterchain_name.c_str(),
                        req.media_name.c_str()
                );
                FilterChain::Ptr filter_chain = filter_chain_mgr_.createFilterChain(req.filterchain_name);

                res.response = detection_task_mgr_.StartDetectionTask(req.media_name, filter_chain, req.node_name);
                ROS_INFO("Starting topic: %s", res.response.c_str());
            }
            catch (const std::exception &e) {
                ROS_ERROR("Starting execution error: %s", e.what());
                res.response = "";
                // Here the call did not fail, we simply deny the service.
                return true;
            }
        } else if (req.cmd == req.STOP) {
            try {
                ROS_INFO(" ");
                ROS_INFO("--- Stoping Execution ---");
                ROS_INFO(
                        "Node: %s, FilterChain: %s, Media: %s",
                        req.node_name.c_str(),
                        req.filterchain_name.c_str(),
                        req.media_name.c_str()
                );

                auto fc = detection_task_mgr_.getFilterChainFromDetectionTask(req.node_name);
                if (fc == nullptr) {
                    ROS_ERROR("FilterChain does not exist, cannot close execution.");
                    return false;
                }

                detection_task_mgr_.stopDetectionTask(req.node_name);

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
            sonia_common::GetInformationList::Request &req,
            sonia_common::GetInformationList::Response &res
    ) {
        if (req.cmd == req.EXEC) {
            res.list = buildRosMessage(detection_task_mgr_.getDetectionTasksNames());
        } else if (req.cmd == req.FILTERCHAIN) {
            res.list = buildRosMessage(proc_image_processing::FilterChainManager::getFilterChainsNames());
        } else if (req.cmd == req.FILTERS) {
            res.list = FilterFactory::getFilters();
        } else if (req.cmd == req.MEDIA) {
            res.list = buildRosMessage(proc_image_processing::DetectionTaskManager::getMediasNames());
        }

        return true;
    }

    bool VisionServer::callbackCopyFilterChain(
            sonia_common::CopyFilterchain::Request &req,
            sonia_common::CopyFilterchain::Response &res
    ) {
        std::ifstream src(
                proc_image_processing::FilterChainManager::getFilterChainPath(req.filterchain_to_copy).c_str(),
                std::ios::binary
        );

        std::ofstream dst(
                proc_image_processing::FilterChainManager::getFilterChainPath(req.filterchain_new_name).c_str(),
                std::ios::binary
        );
        dst << src.rdbuf();
        res.success = res.SUCCESS;
        return true;
    }

    bool VisionServer::callbackGetFilterParam(
            sonia_common::GetFilterchainFilterParam::Request &req,
            sonia_common::GetFilterchainFilterParam::Response &res
    ) {
        res.list = "";

        std::string execution_name(req.exec_name), filter_chain_name(req.filterchain);
        FilterChain::Ptr filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filter_chain != nullptr) {
            auto index = extractFilterIndexFromUIName(req.filter);
            try {
                res.list = filter_chain->getFilterParameterValue(index, req.parameter);
                return true;
            } catch (const std::exception &e) {
                ROS_ERROR("Cannot get parameter %s value for filter at index %zu! Cause: %s", req.parameter.c_str(),
                          index, e.what());
                return false;
            }
        }

        ROS_ERROR(
                "DetectionTask %s does not exist or does not use this filter chain: %s on get filter's param request.",
                execution_name.c_str(),
                filter_chain_name.c_str()
        );
        return false;
    }

    bool VisionServer::callbackGetFilterAllParam(
            sonia_common::GetFilterchainFilterAllParam::Request &req,
            sonia_common::GetFilterchainFilterAllParam::Response &res
    ) {
        res.list = "";

        try
        {
            std::string execution_name(req.exec_name), filter_chain_name(req.filterchain);
            FilterChain::Ptr filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

            if (filter_chain != nullptr) {
                try {
                    auto parameters = filter_chain->getFilterParameters(extractFilterIndexFromUIName(req.filter));
                    std::vector<std::string> parameter_names;
                    for (const auto &parameter : parameters) {
                        parameter_names.push_back(parameter->toString());
                    }
                    res.list = buildRosMessage(parameter_names);
                    return true;
                } catch (const std::exception &e) {
                    ROS_ERROR("Cannot get filter parameters! Cause: %s", e.what());
                    return false;
                }
            }
            ROS_ERROR(
                    "DetectionTask %s does not exist or does not use this filter chain: %s on get filter's param request.",
                    execution_name.c_str(),
                    filter_chain_name.c_str()
            );
        }
        catch(const std::exception& e)
        {
            ROS_ERROR(
                    "callbackGetFilterAllParam DetectionTask %s crash on %s request with the following exception %s",
                    req.exec_name.c_str(),
                    req.filterchain.c_str(),
                    e.what()
            );
        }
        
        
        return false;
    }

    bool VisionServer::callbackSetFilterParam(
            sonia_common::SetFilterchainFilterParam::Request &req,
            sonia_common::SetFilterchainFilterParam::Response &res
    ) {
        res.success = res.FAIL;

        std::string execution_name(req.exec_name), filter_chain_name(req.filterchain);
        FilterChain::Ptr filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filter_chain != nullptr) {
            try {
                filter_chain->setFilterParameterValue(
                        extractFilterIndexFromUIName(req.filter),
                        req.parameter,
                        req.value
                );
                res.success = res.SUCCESS;
                return true;
            } catch (const std::exception &e) {
                ROS_ERROR("Cannot set filter parameter value! Cause: %s", e.what());
                return false;
            }
        }
        ROS_ERROR(
                "DetectionTask %s does not exist or does not use this filter chain: %s on get filter's param request.",
                execution_name.c_str(),
                filter_chain_name.c_str()
        );
        return false;
    }

    bool VisionServer::callbackGetFilter(
            sonia_common::GetFilterchainFilter::Request &req,
            sonia_common::GetFilterchainFilter::Response &res
    ) {
        res.list = "";

        std::string execution_name(req.exec_name), filter_chain_name(req.filterchain);
        FilterChain::Ptr filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filter_chain != nullptr) {
            auto filters = filter_chain->getFilters();
            std::vector<std::string> filter_names;
            for (size_t i = 0; i < filters.size(); ++i) {
                filter_names.push_back(constructFilterUIName(filters.at(i)->getName(), i));
            }
            res.list = buildRosMessage(filter_names);
            return true;
        }

        ROS_ERROR(
                "DetectionTask %s does not exist or does not use this "
                "filter chain: %s on get filter's param request.",
                execution_name.c_str(),
                filter_chain_name.c_str()
        );
        return false;
    }

    bool VisionServer::callbackSetObserver(
            sonia_common::SetFilterchainFilterObserver::Request &req,
            sonia_common::SetFilterchainFilterObserver::Response &res
    ) {
        try
        {
            // For now ignoring filter chain name, but when we will have multiple,
            // we will have to check the name and find the good filter chain
            FilterChain::Ptr filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(req.exec_name);

            if (filter_chain != nullptr) {
                res.result = res.SUCCESS;
                filter_chain->setObserver(extractFilterIndexFromUIName(req.filter));
                return true;
            }

            res.result = res.FAIL;
            ROS_ERROR(
                    "DetectionTask %s does not exist or does not use this filter chain: %s on get filters request",
                    req.exec_name.c_str(),
                    req.filterchain.c_str()
            );
        }
        catch(const std::exception& e)
        {
            ROS_ERROR(
                    "callbackSetObserver DetectionTask %s crash on %s request with the following exception %s",
                    req.exec_name.c_str(),
                    req.filterchain.c_str(),
                    e.what()
            );
        }
        
        
        return false;
    }

    bool VisionServer::callbackManageFilter(
            sonia_common::ManageFilterchainFilter::Request &req,
            sonia_common::ManageFilterchainFilter::Response &res
    ) {
        const auto &filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(req.exec_name);
        res.success = res.FAIL;
        if (filter_chain != nullptr) {
            if (req.cmd == req.ADD) {
                try {
                    filter_chain->addFilter(req.filter);
                    res.success = res.SUCCESS;
                } catch (const std::exception &e) {
                    ROS_ERROR(
                            "Cannot add filter %s to filter chain %s!",
                            req.filter.c_str(),
                            filter_chain->getName().c_str()
                    );
                }
            } else if (req.cmd == req.DELETE) {
                try {
                    filter_chain->removeFilter(extractFilterIndexFromUIName(req.filter));
                    res.success = res.SUCCESS;
                } catch (const std::exception &e) {
                    ROS_ERROR(
                            "Cannot remove filter %s from filter chain %s!",
                            req.filter.c_str(),
                            filter_chain->getName().c_str()
                    );
                }
            }
        }

        return res.success;
    }

    bool VisionServer::callbackManageFilterChain(
            sonia_common::ManageFilterchain::Request &req,
            sonia_common::ManageFilterchain::Response &res
    ) {
        std::string filter_chain_name(req.filterchain);
        res.success = res.FAIL;

        if (req.cmd == req.ADD) {
            proc_image_processing::FilterChainManager::addFilterChain(filter_chain_name);
            res.success = res.SUCCESS;
        } else if (req.cmd == req.DELETE) {
            try {
                proc_image_processing::FilterChainManager::deleteFilterChain(filter_chain_name);
                res.success = res.SUCCESS;
            } catch (const std::exception &e) {
                ROS_ERROR("Cannot delete filter chain %s!", filter_chain_name.c_str());
            }
        }
        return res.success;
    }

    bool VisionServer::callbackSaveFilterChain(
            sonia_common::SaveFilterchain::Request &req,
            sonia_common::SaveFilterchain::Response &res
    ) {
        std::string execution_name(req.exec_name);
        std::string filter_chain_name(req.filterchain);
        if (req.cmd == req.SAVE) {
            res.success = detection_task_mgr_.getFilterChainFromDetectionTask(req.exec_name)->serialize();
        }
        return res.success;
    }

    bool VisionServer::callbackSetFilterChainOrder(
            sonia_common::SetFilterchainFilterOrder::Request &req,
            sonia_common::SetFilterchainFilterOrder::Response &res
    ) {
        ROS_INFO("Call to set_filterchain_filter_order.");

        res.success = res.FAIL;
        auto filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(req.exec_name);
        if (filter_chain != nullptr) {
            try {
                if (req.cmd == req.UP) {
                    filter_chain->moveFilterUp(req.filter_index);
                    res.success = res.SUCCESS;
                    return res.success;
                } else if (req.cmd == req.DOWN) {
                    filter_chain->moveFilterDown(req.filter_index);
                    res.success = res.SUCCESS;
                    return res.success;
                }
            } catch (const std::invalid_argument &e) {
                ROS_ERROR("Cannot move filter! Cause: %s", e.what());
            }
        } else {
            ROS_ERROR("Cannot get filter chain %s!", req.exec_name.c_str());
        }

        return res.success;
    }

    bool VisionServer::callbackGetFilterChainExecution(
            sonia_common::GetFilterchainFromExecution::Request &req,
            sonia_common::GetFilterchainFromExecution::Response &res
    ) {
        ROS_INFO("Call to get_filterchain_from_execution.");
        std::string execution_name(req.exec_name);
        FilterChain::Ptr filter_chain = detection_task_mgr_.getFilterChainFromDetectionTask(execution_name);

        if (filter_chain != nullptr) {
            res.list = filter_chain->getName();
            return true;
        }
        ROS_ERROR(
                "DetectionTask %s does not exist or does not use this filter chain "
                "on get filters request.",
                execution_name.c_str()
        );
        return false;
    }

    bool VisionServer::callbackGetMediaExecution(
            sonia_common::GetMediaFromExecution::Request &req,
            sonia_common::GetMediaFromExecution::Response &res
    ) {
        ROS_INFO("Call to get_media_from_execution.");
        auto response = "media_" + req.exec_name;
        res.list = response;
        return true;
    }

}  // namespace proc_image_processing
