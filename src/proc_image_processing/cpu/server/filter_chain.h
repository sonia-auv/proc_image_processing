/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_
#define PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <queue>

#include <proc_image_processing/cpu/filters/filter.h>
#include "filter_factory.h"
#include "global_param_handler.h"
#include "target.h"

namespace proc_image_processing {

    class FilterChain {
    public:
        using Ptr = std::shared_ptr<FilterChain>;

        explicit FilterChain(const std::string &name);

        explicit FilterChain(const FilterChain &filterChain);

        ~FilterChain();

        /**
         * Get the name of the filterchain.
         *
         * \return The name of the filterchain.
         */
        const std::string &getName() const;

        /**
         * Set the name of the filterchain.
         *
         * \param name The new name of the filterchain.
         */
        void setName(const std::string &name);

        bool serialize();

        bool deserialize();

        proc_image_processing::Filter::Ptr getFilter(const size_t &index) const;

        std::vector<proc_image_processing::Filter::Ptr> getFilters(const std::string &filter_name) const;

        std::vector<proc_image_processing::Filter::Ptr> getFilters() const;

        /**
         * Check if there is a filter with the same name than the given parameter.
         *
         * \param filter_name The name of the filter to check.
         * \return Either if a filter with the same name exists or not.
         */
        bool containsFilter(const std::string &filter_name) const;

        void executeFilterChain(cv::Mat &image);

        void setObserver(const size_t &index);

        void addFilter(const std::string &filter_name);

        void removeFilter(const size_t &index);

        void moveFilterDown(const size_t &filterIndex);

        void moveFilterUp(const size_t &filterIndex);

        std::string getFilterParameterValue(const size_t &index, const std::string &param_name);

        void setFilterParameterValue(const size_t &index,
                                     const std::string &param_name,
                                     const std::string &param_value);

        std::vector<proc_image_processing::ParameterInterface *> getFilterParameters(const size_t &index);

        proc_image_processing::GlobalParamHandler &getParameterHandler();

    private:
        std::string filepath_;

        std::string name_;

        proc_image_processing::GlobalParamHandler param_handler_;

        std::vector<proc_image_processing::Filter::Ptr> filters_;

        size_t observer_index_;
    };

    inline proc_image_processing::Filter::Ptr FilterChain::getFilter(
            const size_t &index) const {
        return filters_.at(index);
    }

    inline std::vector<proc_image_processing::Filter::Ptr>
    FilterChain::getFilters(const std::string &filter_name) const {
        std::vector<proc_image_processing::Filter::Ptr> filters;
        for (const auto &filter : filters_) {
            if (filter->getName() == filter_name) {
                filters.push_back(filter);
            }
        }
        return filters;
    }

    inline std::vector<proc_image_processing::Filter::Ptr> FilterChain::getFilters()
    const {
        return filters_;
    }

    inline bool FilterChain::containsFilter(const std::string &filter_name) const {
        // TODO a map where filters are stored by their name could be better
        for (const auto &filter : filters_) {
            if (filter->getName() == filter_name) {
                return true;
            }
        }
        return false;
    }

    inline void FilterChain::setObserver(const size_t &index) {
        observer_index_ = index;
    }

    inline proc_image_processing::GlobalParamHandler &FilterChain::getParameterHandler() {
        return param_handler_;
    }

    inline const std::string &FilterChain::getName() const { return name_; }

    inline void FilterChain::setName(const std::string &name) { name_ = name; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_
