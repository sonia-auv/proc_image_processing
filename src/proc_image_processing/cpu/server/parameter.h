/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_FILTER_PARAMETER_H_
#define PROVIDER_VISION_FILTER_PARAMETER_H_

#include <sonia_common/macros.h>
#include "parameter_interface.h"
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <string>
#include <vector>

namespace proc_image_processing {

  template <typename Tp_>
  class Parameter : public ParameterInterface {
  public:
    using Ptr = std::shared_ptr<Parameter<Tp_>>;

    static const char SEPARATOR = '|';

    explicit Parameter(const std::string& name, const Tp_& value,
      std::vector<ParameterInterface*>* vector = nullptr,
      const std::string& description = "");

    virtual ~Parameter() = default;

    template <class Ut_>
    bool operator>(const Ut_& rhs) {
      return value_ > rhs;
    }

    template <class Ut_>
    bool operator<(const Ut_& rhs) {
      return value_ < rhs;
    }

    template <class Ut_>
    bool operator==(const Ut_& rhs) {
      return rhs == value_;
    }

    template <class Ut_>
    bool operator!=(const Ut_& rhs) {
      return rhs != value_;
    }

    template <class Ut_>
    void operator+=(const Ut_& rhs) {
      value_ += rhs;
    }

    template <class Ut_>
    void operator++() {
      value_++;
    }

    template <class Ut_>
    void operator-=(const Ut_& rhs) {
      value_ -= rhs;
    }

    template <class Ut_>
    void operator--() {
      value_--;
    }

    template <class Ut_>
    void operator*=(const Ut_& rhs) {
      value_ *= rhs;
    }

    template <class Ut_>
    void operator/=(const Ut_& rhs) {
      value_ /= rhs;
    }

    template <class Ut_>
    int operator+(const Ut_& rhs) {
      return value_ + rhs;
    }

    template <class Ut_>
    int operator-(const Ut_& rhs) {
      return value_ - rhs;
    }

    template <class Ut_>
    int operator*(const Ut_& rhs) {
      return value_ * rhs;
    }

    template <class Ut_>
    int operator/(const Ut_& rhs) {
      return value_ / rhs;
    }

    template <class Ut_>
    void operator=(const Ut_& rhs) {
      SetValue(rhs);
    }

    Tp_ operator()() { return GetValue(); }

    void SetValue(const Tp_& value);

    const Tp_& GetValue() const;

    void SetDescription(const std::string& description) override;

    std::string GetDescription() const override;

    void SetName(const std::string& name) override;

    std::string GetName() const override;

    virtual std::string ToString() const override;

    std::string GetType() const override;

    std::string GetStringValue() const override;

    void SetStringValue(const std::string& value) override;

  protected:
    std::string name_;
    Tp_ value_;
    std::string description_;
  };

}  // namespace proc_image_processing

#include "parameter_inl.h"

#endif  // PROVIDER_VISION_FILTER_PARAMETER_H_
