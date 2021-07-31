#include "filter_chain.h"
#include <yaml-cpp/yaml.h>
#include "proc_image_processing/cpu/config.h"

namespace proc_image_processing {

    FilterChain::FilterChain(const std::string &name)
            : filepath_(kFilterChainPath + "/" + name + kFilterChainExt),
              name_(name),
              param_handler_(),
              observer_index_(0) {
        deserialize();
        observer_index_ = filters_.size() - 1;
    }

    FilterChain::FilterChain(const FilterChain &filter_chain)
            : filepath_(kFilterChainPath + "/" + filter_chain.name_ + "_copy" +
                        kFilterChainExt),
              name_(filter_chain.name_ + "_copy"),
              param_handler_(filter_chain.param_handler_),
              observer_index_(filter_chain.observer_index_) {
    }

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

        auto filepath = kFilterChainPath + "/" + getName() + kFilterChainExt;
        std::ofstream fout(filepath);
        fout << out.c_str();
        return true;
    }

    bool FilterChain::deserialize() {
        YAML::Node node = YAML::LoadFile(filepath_);

        if (node["name"]) {
            setName(node["name"].as<std::string>());
        }

        if (node["filters"]) {
            auto filters = node["filters"];
            assert(filters.Type() == YAML::NodeType::Sequence);

            for (std::size_t i = 0; i < filters.size(); i++) {
                auto filter_node = filters[i];
                addFilter(filter_node["name"].as<std::string>());

                if (filter_node["parameters"]) {
                    auto parameters = filter_node["parameters"];
                    assert(parameters.Type() == YAML::NodeType::Sequence);

                    for (auto &&parameter : parameters) {
                        auto param_node = parameter;

                        auto param_name = param_node["name"].as<std::string>();
                        auto param_value = param_node["value"].as<std::string>();
                        setFilterParameterValue(i, param_name, param_value);
                    }
                }
            }
        }
        return true;
    }

    void FilterChain::executeFilterChain(cv::Mat &image) {
        cv::Mat imageToProcess = image.clone();
        if (!imageToProcess.empty()) {
            param_handler_.setOriginalImage(imageToProcess);

            try {
                size_t index = 0;
                for (auto &filter : filters_) {
                    if (!imageToProcess.empty()) {
                        filter->apply(imageToProcess);
                    }

                    if (index == observer_index_) {
                        imageToProcess.copyTo(image);
                    }

                    index++;
                }
            }
            catch (cv::Exception &e) {
                ROS_ERROR("[FILTERCHAIN %s ], Error in image processing: %s", name_.c_str(), e.what());
            };
        }
    }

    void FilterChain::removeFilter(const size_t &index) {
        if (index <= filters_.size()) {
            auto it = filters_.begin() + index;
            filters_.erase(it);
        }
    }

    void FilterChain::moveFilterDown(const size_t &filterIndex) {
        if (filterIndex < (filters_.size() - 1)) {
            auto itFilter = filters_.begin();
            std::advance(itFilter, filterIndex);

            auto itFilterBellow = filters_.begin();
            std::advance(itFilterBellow, filterIndex + 1);

            std::swap(*itFilter, *itFilterBellow);
        } else {
            std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
            ROS_WARN_NAMED(filterchainID, "Can't move this filter down");
        }
    }

    void FilterChain::moveFilterUp(const size_t &filterIndex) {
        if ((filterIndex > 0) && (filterIndex <= (filters_.size() - 1))) {
            auto itFilter = filters_.begin();
            std::advance(itFilter, filterIndex);

            auto itFilterAbove = filters_.begin();
            std::advance(itFilterAbove, filterIndex - 1);

            std::swap(*itFilter, *itFilterAbove);
        } else {
            std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
            ROS_WARN_NAMED(filterchainID, "Can't move this filter down");
        }
    }

    std::string FilterChain::getFilterParameterValue(const size_t &index, const std::string &name) const {
        return getFilter(index)->getParameterValue(name);
    }

    void FilterChain::setFilterParameterValue(
            const size_t &index,
            const std::string &name,
            const std::string &value
    ) const {
        auto filter = getFilter(index);
        if (filter != nullptr) {
            filter->setParameterValue(name, value);
        }
    }

    std::vector<ParameterInterface *> FilterChain::getFilterParameters(const size_t &index) const {
        return getFilter(index)->getParameters();
    }

    void FilterChain::addFilter(const std::string &filter_name) {
        auto filter = Filter::Ptr(FilterFactory::createInstance(filter_name, param_handler_));
        if (filter != nullptr) {
            filters_.push_back(filter);
        } else {
            throw std::invalid_argument("This filter does not exist in the library");
        }
    }

}  // namespace proc_image_processing
