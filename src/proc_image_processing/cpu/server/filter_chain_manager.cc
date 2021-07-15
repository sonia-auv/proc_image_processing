/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include <proc_image_processing/cpu/server/filter_chain_manager.h>
#include <dirent.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace proc_image_processing {

  const std::string FilterChainManager::FILTER_CHAIN_MANAGER_TAG =
          "FILTERCHAIN_MANAGER";

    FilterChainManager::FilterChainManager() {
    };

    FilterChainManager::~FilterChainManager() {}

    std::vector<std::string> FilterChainManager::getFilterChainsNames() const {
        auto availableFilterchains = std::vector<std::string>{};
        std::stringstream ss;
        ss << kConfigPath << "filterchain/";
        if (auto dir = opendir(ss.str().c_str())) {
            struct dirent *ent;
            while ((ent = readdir(dir)) != nullptr) {
                auto filename = std::string{ent->d_name};
                if (filename.length() > kFilterchainExt.length() &&
                    filename.substr(filename.length() - kFilterchainExt.length()) ==
                    kFilterchainExt) {
                    filename.replace(filename.end() - kFilterchainExt.length(),
                                     filename.end(), "");
                    availableFilterchains.push_back(filename);
                }
            }
            closedir(dir);
        }
        return availableFilterchains;
    }

    void FilterChainManager::addFilterChain(const std::string &filterchain) {
        if (!filterChainExists(filterchain)) {
            YAML::Node node;
            node["name"] = filterchain;

            auto filepath = kFilterchainPath + filterchain + kFilterchainExt;
            std::ofstream fout(filepath);
            fout << node;
        }
    }

    void FilterChainManager::deleteFilterChain(const std::string &filterchain) {
        remove(getFilterChainPath(filterchain).c_str());
    }

    bool FilterChainManager::filterChainExists(const std::string &filterchain) {
        for (const auto &existing_filterchain : getFilterChainsNames()) {
            if (filterchain == existing_filterchain) {
                return true;
            }
        }
        return false;
    }

    FilterChain::Ptr FilterChainManager::createFilterChain(
            const std::string &filterchainName) {
        if (filterChainExists(filterchainName)) {
            auto filterchain = std::make_shared<FilterChain>(filterchainName);
            running_filterchains_.push_back(filterchain);
            ROS_INFO("FilterChain is ready.");
            return filterchain;
        }
        throw std::invalid_argument("Could not find the given filterchain");
    }

    const std::vector<FilterChain::Ptr>
    &FilterChainManager::createAllFilterChains() {
        for (const auto &filterchain : getFilterChainsNames()) {
            createFilterChain(filterchain);
        }
        return getRunningFilterChains();
    }

    void FilterChainManager::stopFilterChain(const FilterChain::Ptr &filterchain) {
        auto instance = std::find(running_filterchains_.begin(),
                                  running_filterchains_.end(), filterchain);
        running_filterchains_.erase(instance);
        ROS_INFO("FilterChain is stopped.");
    }

    std::string FilterChainManager::getFilterChainPath(
            const std::string &filterchain) const {
        return kConfigPath + filterchain + kFilterchainExt;
    }

    const std::vector<FilterChain::Ptr>
    &FilterChainManager::getRunningFilterChains() const {
        return running_filterchains_;
    }

}  // namespace proc_image_processing
