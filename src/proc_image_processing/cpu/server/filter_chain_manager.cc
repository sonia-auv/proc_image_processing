#include "filter_chain_manager.h"
#include <dirent.h>
#include <yaml-cpp/yaml.h>

namespace proc_image_processing {
    // TODO Initialization of 'FILTER_CHAIN_MANAGER_TAG' with static storage duration may throw an exception that cannot be caught
    const std::string FilterChainManager::FILTER_CHAIN_MANAGER_TAG = "FILTERCHAIN_MANAGER";

    FilterChainManager::FilterChainManager() = default;

    FilterChainManager::~FilterChainManager() = default;

    std::vector<std::string> FilterChainManager::getFilterChainsNames() {
        auto availableFilterchains = std::vector<std::string>{};
        std::stringstream ss;
        ss << kConfigPath << "filterchain/";
        if (auto dir = opendir(ss.str().c_str())) {
            struct dirent *ent;
            while ((ent = readdir(dir)) != nullptr) {
                auto filename = std::string{ent->d_name};
                if (filename.length() > kFilterChainExt.length() &&
                    filename.substr(filename.length() - kFilterChainExt.length()) ==
                    kFilterChainExt) {
                    filename.replace(filename.end() - kFilterChainExt.length(),
                                     filename.end(), "");
                    availableFilterchains.push_back(filename);
                }
            }
            closedir(dir);
        }
        return availableFilterchains;
    }

    void FilterChainManager::addFilterChain(const std::string &filter_chain) {
        if (!filterChainExists(filter_chain)) {
            YAML::Node node;
            node["name"] = filter_chain;

            auto filepath = kFilterChainPath + filter_chain + kFilterChainExt;
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

    FilterChain::Ptr FilterChainManager::createFilterChain(const std::string &filterchainName) {
        if (filterChainExists(filterchainName)) {
            auto filterchain = std::make_shared<FilterChain>(filterchainName);
            running_filter_chains_.push_back(filterchain);
            ROS_INFO("FilterChain is ready.");
            return filterchain;
        }
        throw std::invalid_argument("Could not find the given filterchain");
    }

    const std::vector<FilterChain::Ptr> &FilterChainManager::createAllFilterChains() {
        for (const auto &filter_chain : getFilterChainsNames()) {
            createFilterChain(filter_chain);
        }
        return getRunningFilterChains();
    }

    void FilterChainManager::stopFilterChain(const FilterChain::Ptr &filter_chain) {
        auto instance = std::find(running_filter_chains_.begin(),
                                  running_filter_chains_.end(), filter_chain);
        running_filter_chains_.erase(instance);
        ROS_INFO("FilterChain is stopped.");
    }

    std::string FilterChainManager::getFilterChainPath(const std::string &filter_chain) {
        return kConfigPath + filter_chain + kFilterChainExt;
    }

    const std::vector<FilterChain::Ptr>
    &FilterChainManager::getRunningFilterChains() const {
        return running_filter_chains_;
    }

}  // namespace proc_image_processing
