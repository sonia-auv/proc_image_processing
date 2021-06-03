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

#ifndef PROVIDER_VISION_PROC_FILTERCHAIN_H_
#define PROVIDER_VISION_PROC_FILTERCHAIN_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <queue>

#include <proc_image_processing/cpu/filters/filter.h>
#include "filter_factory.h"
#include "global_param_handler.h"
#include "target.h"

namespace proc_image_processing {

class Filterchain {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Filterchain>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Filterchain(const std::string &name);

  explicit Filterchain(const Filterchain &filterchain);

  ~Filterchain();

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Get the name of the filterchain.
   *
   * \return The name of the filterchain.
   */
  const std::string &GetName() const;

  /**
   * Set the name of the filterchain.
   *
   * \param name The new name of the filterchain.
   */
  void SetName(const std::string &name);

  bool Serialize();

  bool Deserialize();

  proc_image_processing::Filter::Ptr GetFilter(const size_t &index) const;

  std::vector<proc_image_processing::Filter::Ptr> GetFiltersWithName(
      const std::string &filter_name) const;

  std::vector<proc_image_processing::Filter::Ptr> GetAllFilters() const;

  /**
   * Check if there is a filter with the same name than the given parameter.
   *
   * \param filter_name The name of the filter to check.
   * \return Either if a filter with the same name exists or not.
   */
  bool ContainsFilter(const std::string &filter_name) const;

  void ExecuteFilterChain(cv::Mat &image);

  void SetObserver(const size_t &index);

  void AddFilter(const std::string &filter_name);

  void RemoveFilter(const size_t &index);

  void MoveFilterDown(const size_t &filterIndex);

  void MoveFilterUp(const size_t &filterIndex);

  std::string GetFilterParameterValue(const size_t &index,
                                      const std::string &param_name);

  void SetFilterParameterValue(const size_t &index,
                               const std::string &param_name,
                               const std::string &param_value);

  std::vector<proc_image_processing::ParameterInterface *> GetFilterAllParameters(
      const size_t &index);

    proc_image_processing::GlobalParamHandler & GetParameterHandler();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string filepath_;

  std::string name_;

  proc_image_processing::GlobalParamHandler param_handler_;

  std::vector<proc_image_processing::Filter::Ptr> filters_;

  size_t observer_index_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline proc_image_processing::Filter::Ptr Filterchain::GetFilter(
    const size_t &index) const {
  return filters_.at(index);
}

//------------------------------------------------------------------------------
//
inline std::vector<proc_image_processing::Filter::Ptr>
Filterchain::GetFiltersWithName(const std::string &filter_name) const {
  std::vector<proc_image_processing::Filter::Ptr> filters;
  for (const auto &filter : filters_) {
    if (filter->GetName() == filter_name) {
      filters.push_back(filter);
    }
  }
  return filters;
}

//------------------------------------------------------------------------------
//
inline std::vector<proc_image_processing::Filter::Ptr> Filterchain::GetAllFilters()
    const {
  return filters_;
}

//------------------------------------------------------------------------------
//
inline bool Filterchain::ContainsFilter(const std::string &filter_name) const {
  for (const auto &filter : filters_) {
    if (filter->GetName() == filter_name) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void Filterchain::SetObserver(const size_t &index) {
  observer_index_ = index;
}

//------------------------------------------------------------------------------
//
inline proc_image_processing::GlobalParamHandler & Filterchain::GetParameterHandler() {
  return param_handler_;
}

//------------------------------------------------------------------------------
//
inline const std::string &Filterchain::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
inline void Filterchain::SetName(const std::string &name) { name_ = name; }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_PROC_FILTERCHAIN_H_
