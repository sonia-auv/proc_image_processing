/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_FILTER_PARAMETER_H_
#define PROC_IMAGE_PROCESSING_FILTER_PARAMETER_H_

#include <sonia_common/macros.h>
#include "parameter_interface.h"
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <string>
#include <vector>

namespace proc_image_processing {

    template<typename Tp_>
    class Parameter : public ParameterInterface {
    public:
        using Ptr = std::shared_ptr<Parameter<Tp_>>;

        static const char SEPARATOR = '|';

        explicit Parameter(std::string name, const Tp_ &value,
                           std::vector<ParameterInterface *> *vector = nullptr,
                           std::string description = "");

        ~Parameter() override = default;

        template<class Ut_>
        bool operator>(const Ut_ &rhs) {
            return value_ > rhs;
        }

        template<class Ut_>
        bool operator<(const Ut_ &rhs) {
            return value_ < rhs;
        }

        template<class Ut_>
        bool operator==(const Ut_ &rhs) {
            return rhs == value_;
        }

        template<class Ut_>
        bool operator!=(const Ut_ &rhs) {
            return rhs != value_;
        }

        template<class Ut_>
        void operator+=(const Ut_ &rhs) {
            value_ += rhs;
        }

        template<class Ut_>
        void operator++() {
            value_++;
        }

        template<class Ut_>
        void operator-=(const Ut_ &rhs) {
            value_ -= rhs;
        }

        template<class Ut_>
        void operator--() {
            value_--;
        }

        template<class Ut_>
        void operator*=(const Ut_ &rhs) {
            value_ *= rhs;
        }

        template<class Ut_>
        void operator/=(const Ut_ &rhs) {
            value_ /= rhs;
        }

        template<class Ut_>
        int operator+(const Ut_ &rhs) {
            return value_ + rhs;
        }

        template<class Ut_>
        int operator-(const Ut_ &rhs) {
            return value_ - rhs;
        }

        template<class Ut_>
        int operator*(const Ut_ &rhs) {
            return value_ * rhs;
        }

        template<class Ut_>
        int operator/(const Ut_ &rhs) {
            return value_ / rhs;
        }

        template<class Ut_>
        void operator=(const Ut_ &rhs) {
            setStringValue(rhs);
        }

        Tp_ operator()() { return getValue(); }

        void setValue(const Tp_ &value);

        const Tp_ &getValue() const;

        void setDescription(const std::string &description) override;

        std::string getDescription() const override;

        void setName(const std::string &name) override;

        std::string getName() const override;

        std::string toString() const override;

        std::string getType() const override;

        std::string getStringValue() const override;

        void setStringValue(const std::string &value) override;

    protected:
        std::string name_;
        Tp_ value_;
        std::string description_;
    };

}  // namespace proc_image_processing

#include "parameter_inl.h"

#endif  // PROC_IMAGE_PROCESSING_FILTER_PARAMETER_H_
