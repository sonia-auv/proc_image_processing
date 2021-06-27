/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_PROC_FILTERCHAIN_H_
#define PROVIDER_VISION_PROC_FILTERCHAIN_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <queue>

#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/filter_factory.h>
#include <proc_image_processing/server/global_param_handler.h>
#include "proc_image_processing/server/target.h"

namespace proc_image_processing {

  class Filterchain {
  public:
    using Ptr = std::shared_ptr<Filterchain>;

    explicit Filterchain(const std::string& name);

    explicit Filterchain(const Filterchain& filterchain);

    ~Filterchain();

    /**
     * Get the name of the filterchain.
     *
     * \return The name of the filterchain.
     */
    const std::string& GetName() const;

    /**
     * Set the name of the filterchain.
     *
     * \param name The new name of the filterchain.
     */
    void SetName(const std::string& name);

    bool Serialize();

    bool Deserialize();

    proc_image_processing::IFilter::Ptr GetFilter(const size_t& index) const;

    std::vector<proc_image_processing::IFilter::Ptr> GetFiltersWithName(
      const std::string& filter_name) const;

    std::vector<proc_image_processing::IFilter::Ptr> GetAllFilters() const;

    /**
     * Check if there is a filter with the same name than the given parameter.
     *
     * \param filter_name The name of the filter to check.
     * \return Either if a filter with the same name exists or not.
     */
    bool ContainsFilter(const std::string& filter_name) const;

    void ExecuteFilterChain(cv::Mat& image);

    void SetObserver(const size_t& index);

    void AddFilter(const std::string& filter_name);

    void RemoveFilter(const size_t& index);

    void MoveFilterDown(const size_t& filterIndex);

    void MoveFilterUp(const size_t& filterIndex);

    std::string GetFilterParameterValue(const size_t& index,
      const std::string& param_name);

    void SetFilterParameterValue(const size_t& index,
      const std::string& param_name,
      const std::string& param_value);

    std::vector<proc_image_processing::ParameterInterface*> GetParametersByFilterIndex(
      const size_t& index);

    proc_image_processing::GlobalParamHandler& GetParameterHandler();

  private:
    std::string filepath_;

    std::string name_;

    proc_image_processing::GlobalParamHandler param_handler_;

    std::vector<proc_image_processing::IFilter::Ptr> filters_;

    size_t observer_index_;
  };

  inline proc_image_processing::IFilter::Ptr Filterchain::GetFilter(
    const size_t& index) const {
    return filters_.at(index);
  }

  inline std::vector<proc_image_processing::IFilter::Ptr>
    Filterchain::GetFiltersWithName(const std::string& filter_name) const {
    std::vector<proc_image_processing::IFilter::Ptr> filters;
    for (const auto& filter : filters_) {
      if (filter->GetName() == filter_name) {
        filters.push_back(filter);
      }
    }
    return filters;
  }

  inline std::vector<proc_image_processing::IFilter::Ptr> Filterchain::GetAllFilters()
    const {
    return filters_;
  }

  inline bool Filterchain::ContainsFilter(const std::string& filter_name) const {
    for (const auto& filter : filters_) {
      if (filter->GetName() == filter_name) {
        return true;
      }
    }
    return false;
  }

  inline void Filterchain::SetObserver(const size_t& index) {
    observer_index_ = index;
  }

  inline proc_image_processing::GlobalParamHandler& Filterchain::GetParameterHandler() {
    return param_handler_;
  }

  inline const std::string& Filterchain::GetName() const { return name_; }

  inline void Filterchain::SetName(const std::string& name) { name_ = name; }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_PROC_FILTERCHAIN_H_
