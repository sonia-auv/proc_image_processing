#ifndef PROC_IMAGE_PROCESSING_FILTER_PARAMETER_H_
#error This file may only be included from parameter.h
#endif

#include <cxxabi.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace proc_image_processing {

    namespace details {

        template<typename Tp_>
        struct StringConvertor {
            static std::string getTypeName() {
                int status;
                std::string tname = typeid(Tp_).name();
                char *demangled_name =
                        abi::__cxa_demangle(tname.c_str(), nullptr, nullptr, &status);
                if (status == 0) {
                    tname = demangled_name;
                    std::free(demangled_name);
                }
                return tname;
            }

            static std::string getString(const Tp_ &value) {
                return std::to_string(value);
            }

            static Tp_ getValue(const std::string &value) {
                return static_cast<Tp_>(value);
            }
        };

        template<>
        struct StringConvertor<int> {
            static std::string getTypeName() { return "Integer"; }

            static std::string getString(const int &value) {
                return std::to_string(value);
            }

            static int getValue(const std::string &value) { return atoi(value.c_str()); }
        };

        template<>
        struct StringConvertor<bool> {
            static std::string getTypeName() { return "Boolean"; }

            static std::string getString(const bool &value) {
                if (value) {
                    return "1";
                } else {
                    return "0";
                }
            }

            static bool getValue(const std::string &value) {
                auto string_cpy = value;
                std::transform(string_cpy.begin(), string_cpy.end(), string_cpy.begin(),
                               ::tolower);
                if (string_cpy == "true" || string_cpy == "1") {
                    return true;
                } else if (string_cpy == "false" || string_cpy == "0") {
                    return false;
                }
                throw std::invalid_argument("Could not convert argument to boolean");
            }
        };

        template<>
        struct StringConvertor<double> {
            static std::string getTypeName() { return "Double"; }

            static std::string getString(const double &value) {
                return std::to_string(value);
            }

            static double getValue(const std::string &value) {
                return atof(value.c_str());
            }
        };

        template<>
        struct StringConvertor<std::string> {
            static std::string getTypeName() { return "String"; }

            static std::string getString(const std::string &value) { return value; }

            static std::string getValue(const std::string &value) { return value; }
        };

    }  // namespace details

    template<class Tp_>
    inline Parameter<Tp_>::Parameter(
            std::string name, const Tp_ &value,
            std::vector<ParameterInterface *> *vector, std::string description)
            : name_(std::move(name)), value_(value), description_(std::move(description)) {
        if (vector != nullptr) {
            vector->push_back(dynamic_cast<ParameterInterface *>(this));
        }
    }

    template<class Tp_>
    inline void Parameter<Tp_>::setDescription(
            const std::string &description) {
        description_ = description;
    }

    template<class Tp_>
    inline std::string Parameter<Tp_>::getDescription() const {
        return description_;
    }

    template<class Tp_>
    inline void Parameter<Tp_>::setName(const std::string &name) {
        name_ = name;
    }

    template<class Tp_>
    inline std::string Parameter<Tp_>::getName() const {
        return name_;
    }

    template<class Tp_>
    inline std::string Parameter<Tp_>::toString() const {
        std::stringstream ss;
        ss << getName() << SEPARATOR;
        ss << getType() << SEPARATOR;
        ss << getStringValue() << SEPARATOR;
        ss << SEPARATOR << SEPARATOR;
        ss << getDescription();
        return ss.str();
    }

    template<class Tp_>
    inline void Parameter<Tp_>::setValue(const Tp_ &value) {
        value_ = value;
    }

    template<class Tp_>
    inline const Tp_ &Parameter<Tp_>::getValue() const {
        return value_;
    }

    template<class Tp_>
    inline std::string Parameter<Tp_>::getType() const {
        return details::StringConvertor<Tp_>::getTypeName();
    }

    template<class Tp_>
    inline std::string Parameter<Tp_>::getStringValue() const {
        return details::StringConvertor<Tp_>::getString(value_);
    }

    template<class Tp_>
    inline void Parameter<Tp_>::setStringValue(const std::string &value) {
        value_ = details::StringConvertor<Tp_>::getValue(value);
    }

}  // namespace proc_image_processing
