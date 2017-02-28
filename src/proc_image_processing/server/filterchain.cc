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

#include "proc_image_processing/server/filterchain.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "proc_image_processing/config.h"

namespace proc_image_processing {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Filterchain::Filterchain(const std::string &name)
    : filepath_(kFilterchainPath + name + kFilterchainExt),
      name_(name),
      param_handler_(),
      observer_index_(0) {
  Deserialize();
  observer_index_ = filters_.size() - 1;
}

//------------------------------------------------------------------------------
//
Filterchain::Filterchain(const Filterchain &filterchain)
    : filepath_(kFilterchainPath + filterchain.name_ + "_copy" +
                kFilterchainExt),
      name_(filterchain.name_ + "_copy"),
      param_handler_(filterchain.param_handler_),
      observer_index_(filterchain.observer_index_) {}

//------------------------------------------------------------------------------
//
Filterchain::~Filterchain() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool Filterchain::Serialize() {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "name";
  out << YAML::Value << GetName();

  if (filters_.size() > 0) {
    out << YAML::Key << "filters";
    out << YAML::Value << YAML::BeginSeq;
    for (auto &filter : filters_) {
      out << YAML::BeginMap;
      out << YAML::Key << "name";
      out << YAML::Value << filter->GetName();

      auto parameters = filter->GetParameters();
      if (parameters.size() > 0) {
        out << YAML::Key << "parameters";
        out << YAML::Value << YAML::BeginSeq;
        for (const auto &parameter : parameters) {
          out << YAML::BeginMap;
          out << YAML::Key << "name";
          out << YAML::Value << parameter->GetName();
          out << YAML::Key << "value";
          out << YAML::Value << parameter->GetStringValue();
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;
      }
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;

  auto filepath = kFilterchainPath + GetName() + kFilterchainExt;
  std::ofstream fout(filepath);
  fout << out.c_str();
  return true;
}

//------------------------------------------------------------------------------
//
bool Filterchain::Deserialize() {
  YAML::Node node = YAML::LoadFile(filepath_);

  if (node["name"]) {
    SetName(node["name"].as<std::string>());
  }

  if (node["filters"]) {
    auto filters = node["filters"];
    assert(filters.Type() == YAML::NodeType::Sequence);

    for (std::size_t i = 0; i < filters.size(); i++) {
      auto filter_node = filters[i];
      AddFilter(filter_node["name"].as<std::string>());

      if (filter_node["parameters"]) {
        auto parameters = filter_node["parameters"];
        assert(parameters.Type() == YAML::NodeType::Sequence);

        for (std::size_t j = 0; j < parameters.size(); j++) {
          auto param_node = parameters[j];

          auto param_name = param_node["name"].as<std::string>();
          auto param_value = param_node["value"].as<std::string>();
          SetFilterParameterValue(i, param_name, param_value);
        }
      }
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
void Filterchain::ExecuteFilterChain(cv::Mat &image) {
  cv::Mat imageToProcess = image.clone();
  if (!imageToProcess.empty()) {
    param_handler_.setOriginalImage(imageToProcess);

    auto it = filters_.begin();
    try {
      size_t index = 0;
      for (; it != filters_.end(); ++it) {
        if (!imageToProcess.empty()) {
          (*it)->Execute(imageToProcess);
        }

        if (index == observer_index_) {
          imageToProcess.copyTo(image);
        }

        index++;
      }
    } catch (cv::Exception &e)
    {
      ROS_ERROR("[FILTERCHAIN %s ], Error in image processing: %s",name_, e.what());
    };
  }
}

//------------------------------------------------------------------------------
//
void Filterchain::RemoveFilter(const size_t &index) {
  if (index <= filters_.size()) {
    auto it = filters_.begin() + index;
    filters_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
void Filterchain::MoveFilterDown(const size_t &index) {
  if (index < (filters_.size() - 1)) {
    auto itFilter = filters_.begin();
    std::advance(itFilter, index);

    auto itFilterBellow = filters_.begin();
    std::advance(itFilterBellow, index + 1);

    std::swap(*itFilter, *itFilterBellow);
  } else {
    std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
    ROS_WARN_NAMED(filterchainID.c_str(), "Can't move this filter down");
  }
}

//------------------------------------------------------------------------------
//
void Filterchain::MoveFilterUp(const size_t &index) {
  if ((index > 0) && (index <= (filters_.size() - 1))) {
    auto itFilter = filters_.begin();
    std::advance(itFilter, index);

    auto itFilterAbove = filters_.begin();
    std::advance(itFilterAbove, index - 1);

    std::swap(*itFilter, *itFilterAbove);
  } else {
    std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
    ROS_WARN_NAMED(filterchainID.c_str(), "Can't move this filter down");
  }
}

//------------------------------------------------------------------------------
//
std::string Filterchain::GetFilterParameterValue(
    const size_t &index, const std::string &param_name) {
  return GetFilter(index)->GetParameterValue(param_name);
}

//------------------------------------------------------------------------------
//
void Filterchain::SetFilterParameterValue(const size_t &index,
                                          const std::string &param_name,
                                          const std::string &param_value) {
  GetFilter(index)->SetParameterValue(param_name, param_value);
}

//------------------------------------------------------------------------------
//
std::vector<proc_image_processing::ParameterInterface *>
Filterchain::GetFilterAllParameters(const size_t &index) {
  return GetFilter(index)->GetParameters();
}

//------------------------------------------------------------------------------
//
void Filterchain::AddFilter(const std::string &filter_name) {
  auto filter = proc_image_processing::Filter::Ptr(
      proc_image_processing::FilterFactory::createInstance(filter_name,
                                                     param_handler_));
  if (filter != nullptr) {
    filters_.push_back(filter);
  } else {
    throw std::invalid_argument("This filter does not exist in the library");
  }
}

}  // namespace proc_image_processing
