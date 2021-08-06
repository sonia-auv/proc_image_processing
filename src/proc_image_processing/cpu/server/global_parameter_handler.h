#ifndef PROC_IMAGE_PROCESSING_FILTER_GLOBAL_PARAM_HANDLER_H_
#define PROC_IMAGE_PROCESSING_FILTER_GLOBAL_PARAM_HANDLER_H_

#include "parameter.h"
#include "ranged_parameter.h"
#include "target.h"
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <map>

namespace proc_image_processing {

    class GlobalParameterHandler {
    public:
        using Ptr = std::shared_ptr<GlobalParameterHandler>;

        explicit GlobalParameterHandler() : target_queue_(), params_(), original_image_() {}

        // Since we erase everything, it is easier to delete objet first
        // then calling clear method, since erase invalidate pointer AND
        // needs an iterator. When erasing, the vector replace object so
        // that they are consecutive in memory... on long vector it
        // is "very" long to do. To prevent that, we can use reverse
        // iterator, but erase does not take it...
        ~GlobalParameterHandler() { params_.clear(); }

        // Original image handling
        // WE WANT TO RETURN A COPY, ELSE THE IMAGE WILL BE ALTERATE
        inline cv::Mat getOriginalImage() {
            // Here the return makes a copy so we are safe.
            return original_image_;
        }

        // WE WANT A COPY BECAUSE THE ORIGINAL IMAGE IS PROBABLY GOING TO BE ALTERED
        // BY THE FILTERS.
        inline void setOriginalImage(cv::Mat &image) { original_image_ = image; }

        // Target related
        inline void addTarget(const Target &target) { target_queue_.push(target); }

        // REturns reference so we can pop when we read.
        inline TargetQueue &getTargetQueue() { return target_queue_; }

        inline void clearTarget() {
            while (!target_queue_.empty()) {
                target_queue_.pop();
            }
        }

        /**
         * Add a parameter
         * @param param the parameter
         * @throws std::invalid_argument if the parameter name already exists
         */
        inline void addParameter(ParameterInterface *param) {
            if (param != nullptr) {
                if (params_.find(param->getName()) != params_.end()) {
                    throw std::invalid_argument("A parameter with the same name already exists!");
                }
                params_.emplace(std::pair<std::string, ParameterInterface *>(param->getName(), param));
            }
        }

        void removeParameter(const std::string &name) {
            auto i = params_.find(name);
            if (i != params_.end()) {
                params_.erase(i);
            }
        }

        inline ParameterInterface *getParameter(const std::string &name) const {
            auto i = params_.find(name);
            if (i != params_.end()) {
                return i->second;
            }
            return nullptr;
        }

        // Util
        static const char SEPARATOR = ';';

    private:
        std::queue<Target> target_queue_;
        // Using pointer here to preserve the object if
        // the vector is moved in memory.
        std::map<std::string, ParameterInterface *> params_;
        cv::Mat original_image_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTER_GLOBAL_PARAM_HANDLER_H_
