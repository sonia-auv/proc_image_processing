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

#include "proc_image_processing/server/filterchain_manager.h"
#include <dirent.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "proc_image_processing/ChangeNetwork.h"

namespace proc_image_processing {

const std::string FilterchainManager::FILTERCHAIN_MANAGER_TAG =
    "FILTERCHAIN_MANAGER";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
FilterchainManager::FilterchainManager(){
  deep_network_service = ros::NodeHandle("~").serviceClient<ChangeNetwork>("/deep_detector/change_network");
};

//------------------------------------------------------------------------------
//
FilterchainManager::~FilterchainManager() {}

//==============================================================================
// M E T H O D   S E C T I O N
//------------------------------------------------------------------------------
//
std::vector<std::string> FilterchainManager::GetAllFilterchainName() const {
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

//------------------------------------------------------------------------------
//
void FilterchainManager::CreateFilterchain(const std::string &filterchain) {
  if (!FilterchainExists(filterchain)) {
    YAML::Node node;
    node["name"] = filterchain;

    auto filepath = kFilterchainPath + filterchain + kFilterchainExt;
    std::ofstream fout(filepath);
    fout << node;
  }
}

//------------------------------------------------------------------------------
//
void FilterchainManager::EraseFilterchain(const std::string &filterchain) {
  remove(GetFilterchainPath(filterchain).c_str());
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::FilterchainExists(const std::string &filterchain) {
  for (const auto &existing_filterchain : GetAllFilterchainName()) {
    if (filterchain == existing_filterchain) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
Filterchain::Ptr FilterchainManager::InstanciateFilterchain(
    const std::string &filterchainName) {
  if (FilterchainExists(filterchainName)) {
    ChangeNetwork network;
    network.request.task = filterchainName;
    deep_network_service.call(network);
    auto filterchain = std::make_shared<Filterchain>(filterchainName);
    running_filterchains_.push_back(filterchain);
    ROS_INFO("Filterchain is ready.");
    return filterchain;
  }
  throw std::invalid_argument("Could not find the given filterchain");
}

//------------------------------------------------------------------------------
//
const std::vector<Filterchain::Ptr>
    &FilterchainManager::InstanciateAllFilterchains() {
  for (const auto &filterchain : GetAllFilterchainName()) {
    InstanciateFilterchain(filterchain);
  }
  return GetRunningFilterchains();
}

//------------------------------------------------------------------------------
//
void FilterchainManager::StopFilterchain(const Filterchain::Ptr &filterchain) {
  auto instance = std::find(running_filterchains_.begin(),
                            running_filterchains_.end(), filterchain);
  running_filterchains_.erase(instance);
  ROS_INFO("Filterchain is stopped.");
}

//------------------------------------------------------------------------------
//
std::string FilterchainManager::GetFilterchainPath(
    const std::string &filterchain) const {
  return kConfigPath + filterchain + kFilterchainExt;
}

//------------------------------------------------------------------------------
//
const std::vector<Filterchain::Ptr>
    &FilterchainManager::GetRunningFilterchains() const {
  return running_filterchains_;
}

}  // namespace proc_image_processing
