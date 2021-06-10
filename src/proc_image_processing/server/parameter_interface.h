/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_PARAMETER_INTERFACE_H_
#define PROVIDER_VISION_PARAMETER_INTERFACE_H_

#include <sonia_common/macros.h>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <string>

namespace proc_image_processing {

  /**
   * This is the public interface of a parameter. We want to use an interface
   * because we have to store the list of the parameter in a container.
   * We cannot store the templates pointers, in this case, we will simply store
   * this interface and apply polymorphism on the different methods.
   */
  class ParameterInterface {
  public:
    using Ptr = std::shared_ptr<ParameterInterface>;

    ParameterInterface() = default;

    virtual ~ParameterInterface() = default;

    virtual void SetDescription(const std::string& description) = 0;

    virtual std::string GetDescription() const = 0;

    virtual void SetName(const std::string& name) = 0;

    virtual std::string GetName() const = 0;

    virtual std::string ToString() const = 0;

    virtual std::string GetType() const = 0;

    virtual std::string GetStringValue() const = 0;

    virtual void SetStringValue(const std::string&) = 0;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_PARAMETER_INTERFACE_H_
