#ifndef PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_
#define PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <queue>

#include <proc_image_processing/cpu/filters/filter.h>
#include "filter_factory.h"
#include "global_parameter_handler.h"
#include "target.h"

namespace proc_image_processing {

    class FilterChain {
    public:
        using Ptr = std::shared_ptr<FilterChain>;

        explicit FilterChain(const std::string &name);

        /**
         * Instantiate a filter chain from another one. In other words, this is a copy constructor.
         * @param filter_chain the filter chain from which to copy
         */
        FilterChain(const FilterChain &filter_chain);

        ~FilterChain();

        const std::string &getName() const;

        void setName(const std::string &name);

        void setFilepath(const std::string &filepath);

        /**
         * Save current filter chain as a YAML config file
         * @return false if the operation was a success
         */
        bool serialize();

        /**
         * Load filter chain YAML config file
         * @return true if the operation was a success
         */
        bool deserialize();

        /**
         * Get filter from it's index in the filter chain
         * @param index the filter's index
         * @return nullptr if outside the index range
         */
        Filter::Ptr getFilter(const size_t &index) const;

        /**
         * Get all filters with same name
         * @param filter_name
         * @return a map where it's key is the index of the filter and it's value the filter
         */
        std::map<int, Filter::Ptr> getFilters(const std::string &filter_name) const;

        /**
         * Get all filters from the filter chain
         * @return a vector with all the filters pointers
         */
        std::vector<Filter::Ptr> getFilters() const;

        /**
         * Check if filter chain contains a particular type of filter
         * @param name the filter's name
         * @return true if found
         */
        bool containsFilter(const std::string &name) const;

        /**
         * Apply the filter chain onto the image
         * @param image the image on which to apply the filter chain
         */
        void applyFilterChain(cv::Mat &image);

        void setObserver(const size_t &index);

        /**
         * Add a filter to the filter chain
         * @param name the filter's name
         * @throws std::invalid_argument if filter name is invalid
         */
        void addFilter(const std::string &name);

        /**
         * Remove a filter from the filter chain
         * @param index the filter's index
         * @throws std::invalid_argument if the filter's index is out of range
         */
        void removeFilter(const size_t &index);

        /**
         * Move a filter towards the beginning of the filter chain (towards the first filter)
         * @param index the filter's index to move
         * @throws std::invalid_argument if the filter is already at the end
         */
        void moveFilterDown(const size_t &index);

        /**
         * Move a filter towards the end of the filter chain (towards the last filter)
         * @param index the filter's index to move
         * @throws std::invalid_argument if the filter is already at the beginning
         */
        void moveFilterUp(const size_t &index);

        /**
         * Get a filter's parameter value
         * @param index the filter's index
         * @param name the parameter's name
         * @return the parameter's value
         * @throws std::invalid_argument if the filter's index is invalid
         */
        std::string getFilterParameterValue(const size_t &index, const std::string &name) const;

        /**
         * Set a filter's parameter value
         * @param index the filter's index
         * @param name the parameter's name
         * @param value the parameter's value
         * @throws std::invalid_argument if the filter's index is invalid
         */
        void setFilterParameterValue(const size_t &index, const std::string &name, const std::string &value) const;

        /**
         * Get a filter's parameters
         * @param index the filter's index in the filter chain
         * @return the parameters
         * @throws std::invalid_argument if filter cannot be found
         */
        std::vector<ParameterInterface *> getFilterParameters(const size_t &index) const;

        GlobalParamHandler &getParameterHandler();

    private:
        std::string filepath_;

        std::string name_;

        GlobalParamHandler param_handler_;

        std::vector<Filter::Ptr> filters_;

        size_t observer_index_;
    };

    inline Filter::Ptr FilterChain::getFilter(const size_t &index) const {
        try {
            return filters_.at(index);
        } catch (const std::out_of_range &e) {
            return nullptr;
        }
    }

    inline std::map<int, Filter::Ptr> FilterChain::getFilters(const std::string &filter_name) const {
        std::map<int, Filter::Ptr> filters;
        for (auto i = 0; i < filters_.size(); i++) {
            if (filters_[i]->getName() == filter_name) {
                filters.emplace(i, filters_[i]);
            }
        }
        return filters;
    }

    inline std::vector<Filter::Ptr> FilterChain::getFilters() const { return filters_; }

    inline bool FilterChain::containsFilter(const std::string &name) const {
        return std::any_of(
                filters_.begin(),
                filters_.end(),
                [name](auto f) { return f->getName() == name; }
        );
    }

    inline void FilterChain::setObserver(const size_t &index) { observer_index_ = index; }

    inline GlobalParamHandler &FilterChain::getParameterHandler() { return param_handler_; }

    inline const std::string &FilterChain::getName() const { return name_; }

    inline void FilterChain::setName(const std::string &name) { name_ = name; }

    inline void FilterChain::setFilepath(const std::string &filepath) { filepath_ = filepath; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_PROC_FILTER_CHAIN_H_
