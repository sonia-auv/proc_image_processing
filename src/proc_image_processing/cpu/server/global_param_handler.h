/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_FILTER_GLOBAL_PARAM_HANDLER_H_
#define PROVIDER_VISION_FILTER_GLOBAL_PARAM_HANDLER_H_

#include "parameter.h"
#include "ranged_parameter.h"
#include "target.h"
#include <memory>
#include <queue>
#include <sstream>
#include <string>

namespace proc_image_processing {

  class GlobalParamHandler {
  public:
    using Ptr = std::shared_ptr<GlobalParamHandler>;

    explicit GlobalParamHandler()
      : _vision_target(), _params_vec(), _original_image() {
    }

    // Since we erase everything, it is easier to delete objet first
    // then calling clear method, since erase invalidate pointer AND
    // needs an iterator. When erasing, the vector replace object so
    // that they are consecutive in memory... on long vector it
    // is "very" long to do. To prevent that, we can use reverse
    // iterator, but erase does not take it...
    ~GlobalParamHandler() { _params_vec.clear(); }

    // Original image handling
    // WE WANT TO RETURN A COPY, ELSE THE IMAGE WILL BE ALTERATE
    inline cv::Mat getOriginalImage() {
      // Here the return makes a copy so we are safe.
      return _original_image;
    }

    // WE WANT A COPY BECAUSE THE ORIGINAL IMAGE IS PROBABLY GOING TO BE ALTERED
    // BY THE FILTERS.
    inline void setOriginalImage(cv::Mat image) { _original_image = image; }

    // Target related
    inline void addTarget(const Target& target) { _vision_target.push(target); }

    // REturns reference so we can pop when we read.
    inline TargetQueue& getTargetQueue() { return _vision_target; }

    inline void clearTarget() {
      while (!_vision_target.empty()) {
        _vision_target.pop();
      }
    }

    // Params
    inline void addParam(ParameterInterface* param) {
      _params_vec.push_back(param);
    }

    void removeParam(const std::string& name) {
      // Using iterator as it is simpler to erase.
      std::vector<ParameterInterface*>::iterator index = _params_vec.begin();
      std::vector<ParameterInterface*>::const_iterator end = _params_vec.end();
      bool deleted = false;
      for (; index != end && !deleted; index++) {
        if ((*index)->GetName() == name) {
          _params_vec.erase(index);
          deleted = true;
        }
      }
    }

    inline ParameterInterface* getParam(const std::string& name) const {
      // Using [] accessor for optimisation.
      for (size_t i = 0, size = _params_vec.size(); i < size; i++) {
        if (_params_vec[i]->GetName() == name) {
          return _params_vec[i];
        }
      }
      return 0;
    }

    // Util
    static const char SEPARATOR = ';';

  private:
    std::queue<Target> _vision_target;
    // Using pointer here to preserve the object if
    // the vector is moved in memory.
    std::vector<ParameterInterface*> _params_vec;
    cv::Mat _original_image;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTER_GLOBAL_PARAM_HANDLER_H_
