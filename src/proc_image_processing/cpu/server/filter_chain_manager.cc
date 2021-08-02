#include "filter_chain_manager.h"
#include "boost/filesystem.hpp"
#include <dirent.h>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

namespace proc_image_processing {
    // TODO Initialization of 'FILTER_CHAIN_MANAGER_TAG' with static storage duration may throw an exception that cannot be caught
    const std::string FilterChainManager::FILTER_CHAIN_MANAGER_TAG = "FILTERCHAIN_MANAGER";

    FilterChainManager::FilterChainManager() = default;

    FilterChainManager::~FilterChainManager() = default;

    std::vector<std::string> FilterChainManager::getFilterChainsNames() {
        std::vector<std::string> filter_chains;
        auto filter_chains_dir(kFilterChainPath);
        if (fs::is_directory(filter_chains_dir)) {
            for (fs::path file : fs::directory_iterator(filter_chains_dir)) {
                if (file.extension().string<std::string>() == kFilterChainPath) {
                    filter_chains.push_back(file.filename().string());
                }
            }
        }
        return filter_chains;
    }

    void FilterChainManager::addFilterChain(const std::string &filter_chain) {
        if (!filterChainExists(filter_chain)) {
            YAML::Node node;
            node["name"] = filter_chain;

            auto filepath = kFilterChainPath + "/" + filter_chain + kFilterChainExt;
            std::ofstream fout(filepath);
            fout << node;
        }
    }

    void FilterChainManager::deleteFilterChain(const std::string &filter_chain) {
        remove(getFilterChainPath(filter_chain).c_str());
    }

    bool FilterChainManager::filterChainExists(const std::string &filter_chain) {
        std::vector<std::string> filter_chains_names = getFilterChainsNames();
        return std::any_of(
                filter_chains_names.begin(),
                filter_chains_names.end(),
                [filter_chain](auto n) { return n == filter_chain; }
        );
    }

    FilterChain::Ptr FilterChainManager::createFilterChain(const std::string &filter_chain_name) {
        if (filterChainExists(filter_chain_name)) {
            auto filter_chain = std::make_shared<FilterChain>(filter_chain_name);
            running_filter_chains_.push_back(filter_chain);
            ROS_INFO("Filter chain %s is ready.", filter_chain_name.c_str());
            return filter_chain;
        }
        throw std::invalid_argument("Could not find the given filter chain");
    }

    const std::vector<FilterChain::Ptr> &FilterChainManager::createAllFilterChains() {
        for (const auto &filter_chain : getFilterChainsNames()) {
            createFilterChain(filter_chain);
        }
        return getRunningFilterChains();
    }

    void FilterChainManager::stopFilterChain(const FilterChain::Ptr &filter_chain) {
        auto instance = std::find(
                running_filter_chains_.begin(),
                running_filter_chains_.end(),
                filter_chain
        );
        running_filter_chains_.erase(instance);
        ROS_INFO("FilterChain is stopped.");
    }

    std::string FilterChainManager::getFilterChainPath(const std::string &filter_chain) {
        return kFilterChainPath + "/" + filter_chain + kFilterChainExt;
    }

    const std::vector<FilterChain::Ptr>
    &FilterChainManager::getRunningFilterChains() const {
        return running_filter_chains_;
    }

}  // namespace proc_image_processing
