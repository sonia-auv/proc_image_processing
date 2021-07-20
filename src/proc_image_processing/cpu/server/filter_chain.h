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

        FilterChain(const FilterChain &filter_chain);

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

        Filter::Ptr getFilter(const size_t &index) const;

        std::vector<Filter::Ptr> getFilters(const std::string &filter_name) const;

        std::vector<Filter::Ptr> getFilters() const;

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

        std::string getFilterParameterValue(const size_t &index, const std::string &name) const;

        void setFilterParameterValue(const size_t &index, const std::string &name, const std::string &value) const;

        std::vector<ParameterInterface *> getFilterParameters(const size_t &index) const;

        GlobalParamHandler &getParameterHandler();

    private:
        std::string filepath_;

        std::string name_;

        GlobalParamHandler param_handler_;

        std::vector<Filter::Ptr> filters_;

        size_t observer_index_;
    };

    inline Filter::Ptr FilterChain::getFilter(
            const size_t &index) const {
        return filters_.at(index);
    }

    inline std::vector<Filter::Ptr>
    FilterChain::getFilters(const std::string &filter_name) const {
        std::vector<Filter::Ptr> filters;
        for (const auto &filter : filters_) {
            if (filter->getName() == filter_name) {
                filters.push_back(filter);
            }
        }
        return filters;
    }

    inline std::vector<Filter::Ptr> FilterChain::getFilters()
    const {
        return filters_;
    }

    inline bool FilterChain::containsFilter(const std::string &filter_name) const {
        // TODO a map where filters are stored by their name could be better
        return std::any_of(
                filters_.begin(),
                filters_.end(),
                [filter_name](auto f) { return f->getName() == filter_name; }
        );
    }

    inline void FilterChain::setObserver(const size_t &index) {
        observer_index_ = index;
    }

    inline GlobalParamHandler &FilterChain::getParameterHandler() {
        return param_handler_;
    }

    inline const std::string &FilterChain::getName() const { return name_; }

    inline void FilterChain::setName(const std::string &name) { name_ = name; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_
