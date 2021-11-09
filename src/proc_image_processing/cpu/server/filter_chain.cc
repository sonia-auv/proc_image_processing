#include "filter_chain.h"
#include <yaml-cpp/yaml.h>
#include "proc_image_processing/cpu/config.h"

namespace proc_image_processing {

    FilterChain::FilterChain(std::string name, const std::string &path)
            : filepath_(path + "/" + name + kFilterChainExt),
              name_(std::move(name)),
              param_handler_(),
              observer_index_(0) {
        deserialize();
        observer_index_ = filters_.size() - 1;
    }

    FilterChain::FilterChain(const FilterChain &filter_chain, const std::string &path)
            : filepath_(path + "/" + filter_chain.name_ + "_copy" + kFilterChainExt),
              name_(filter_chain.name_ + "_copy"),
              filters_(filter_chain.getFilters()),
              param_handler_(filter_chain.param_handler_),
              observer_index_(filter_chain.observer_index_) {}

    FilterChain::~FilterChain() = default;

    bool FilterChain::serialize() {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "name";
        out << YAML::Value << getName();

        if (!filters_.empty()) {
            out << YAML::Key << "filters";
            out << YAML::Value << YAML::BeginSeq;
            for (auto &filter : filters_) {
                out << YAML::BeginMap;
                out << YAML::Key << "name";
                out << YAML::Value << filter->getName();

                auto parameters = filter->getParameters();
                if (!parameters.empty()) {
                    out << YAML::Key << "parameters";
                    out << YAML::Value << YAML::BeginSeq;
                    for (const auto &parameter : parameters) {
                        out << YAML::BeginMap;
                        out << YAML::Key << "name";
                        out << YAML::Value << parameter->getName();
                        out << YAML::Key << "value";
                        out << YAML::Value << parameter->getStringValue();
                        out << YAML::EndMap;
                    }
                    out << YAML::EndSeq;
                }
                out << YAML::EndMap;
            }
            out << YAML::EndSeq;
        }
        out << YAML::EndMap;

        // If filter chain is renamed
        auto filepath = kFilterChainPath + "/" + getName() + kFilterChainExt;

        try {
            std::ofstream fout(filepath_);
            fout << out.c_str();
        } catch (std::exception &e) {
            ROS_WARN("Cannot write filter chain %s to %s", getName().c_str(), filepath_.c_str());
            return false;
        }
        return true;
    }

    bool FilterChain::deserialize() {
        try {
            YAML::Node node = YAML::LoadFile(filepath_);

            if (node["name"]) {
                setName(node["name"].as<std::string>());
            }

            if (node["filters"]) {
                auto filters = node["filters"];
                assert(filters.Type() == YAML::NodeType::Sequence);

                // Would duplicate filters if filter chain is deserialized after being instantiated
                if (!filters_.empty()) {
                    filters_.clear();
                }

                for (auto i = 0; i < filters.size(); i++) {
                    auto filter_node = filters[i];
                    addFilter(filter_node["name"].as<std::string>());

                    if (filter_node["parameters"]) {
                        auto parameters = filter_node["parameters"];
                        assert(parameters.Type() == YAML::NodeType::Sequence);

                        for (auto &&parameter : parameters) {
                            auto param_name = parameter["name"].as<std::string>();
                            auto param_value = parameter["value"].as<std::string>();
                            setFilterParameterValue(i, param_name, param_value);
                        }
                    }
                }
            }
            return true;
        } catch (const std::exception &e) {
            ROS_WARN("Cannot load filter chain with path %s. Cause: %s", filepath_.c_str(), e.what());
            return false;
        }
    }

    void FilterChain::applyFilterChain(cv::Mat &image) {
        cv::Mat clone = image.clone();
        if (!clone.empty()) {
            param_handler_.setOriginalImage(clone);

            try {
                size_t index = 0;
                for (auto &filter : filters_) {
                    if (filter != nullptr && !clone.empty()) {
                        filter->execute(clone);
                    }

                    if (index == observer_index_) {
                        clone.copyTo(image);
                    }

                    index++;
                }
            }
            catch (cv::Exception &e) {
                ROS_ERROR("[FILTERCHAIN %s ], Error in image processing: %s", name_.c_str(), e.what());
            }
        }
    }

    void FilterChain::removeFilter(const size_t &index) {
        if (index >= 0 && index < filters_.size()) {
            auto it = filters_.begin() + index;
            filters_.erase(it);
        } else {
            throw std::invalid_argument("Cannot remove filter with an outside range index!");
        }
    }

    void FilterChain::moveFilterDown(const size_t &index) {
        if (index >= 0 && index < (filters_.size() - 1)) {
            auto itFilter = filters_.begin();
            std::advance(itFilter, index);

            auto itFilterBellow = filters_.begin();
            std::advance(itFilterBellow, index + 1);

            std::swap(*itFilter, *itFilterBellow);
        } else {
            throw std::invalid_argument("Cannot move filter down, it is already at the end of the filter chain!");
        }
    }

    void FilterChain::moveFilterUp(const size_t &index) {
        if (index > 0 && index <= (filters_.size() - 1)) {
            auto itFilter = filters_.begin();
            std::advance(itFilter, index);

            auto itFilterAbove = filters_.begin();
            std::advance(itFilterAbove, index - 1);

            std::swap(*itFilter, *itFilterAbove);
        } else {
            throw std::invalid_argument("Cannot move filter up, it is already at the beginning of the filter chain!");
        }
    }

    std::string FilterChain::getFilterParameterValue(const size_t &index, const std::string &name) const {
        auto filter = getFilter(index);
        if (filter != nullptr) {
            return filter->getParameterValue(name);
        }
        throw std::invalid_argument("Cannot fetch " + name + "'s value!");
    }

    void FilterChain::setFilterParameterValue(
            const size_t &index,
            const std::string &name,
            const std::string &value
    ) const {
        auto filter = getFilter(index);
        if (filter != nullptr) {
            filter->setParameterValue(name, value);
        } else {
            throw std::invalid_argument("Cannot set filter parameter value!");
        }
    }

    std::vector<ParameterInterface *> FilterChain::getFilterParameters(const size_t &index) const {
        auto filter = getFilter(index);
        if (filter != nullptr) {
            return filter->getParameters();
        }
        throw std::invalid_argument("Cannot fetch filter parameters!");
    }

    void FilterChain::addFilter(const std::string &name) {
        auto filter = Filter::Ptr(FilterFactory::createInstance(name, param_handler_));
        if (filter != nullptr) {
            filters_.push_back(filter);
        } else {
            throw std::invalid_argument("Filter " + name + " does not exist!");
        }
    }

}  // namespace proc_image_processing
