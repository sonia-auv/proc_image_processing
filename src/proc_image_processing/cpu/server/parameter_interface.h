#ifndef PROC_IMAGE_PROCESSING_PARAMETER_INTERFACE_H_
#define PROC_IMAGE_PROCESSING_PARAMETER_INTERFACE_H_

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

        virtual void setDescription(const std::string &description) = 0;

        virtual std::string getDescription() const = 0;

        virtual void setName(const std::string &name) = 0;

        virtual std::string getName() const = 0;

        virtual std::string toString() const = 0;

        virtual std::string getType() const = 0;

        virtual std::string getStringValue() const = 0;

        virtual void setStringValue(const std::string &) = 0;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_PARAMETER_INTERFACE_H_
