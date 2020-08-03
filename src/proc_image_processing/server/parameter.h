/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#ifndef PROVIDER_VISION_FILTER_PARAMETER_H_
#define PROVIDER_VISION_FILTER_PARAMETER_H_

#include <sonia_common/macros.h>
#include <proc_image_processing/server/parameter_interface.h>
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
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Parameter<Tp_>>;

  static const char SEPARATOR = '|';

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Parameter(const std::string &name, const Tp_ &value,
                     std::vector<ParameterInterface *> *vector = nullptr,
                     const std::string &description = "");

  virtual ~Parameter() = default;

  //============================================================================
  // P U B L I C   O P E R A T O R S

  template <class Ut_>
  bool operator>(const Ut_ &rhs) {
    return value_ > rhs;
  }

  template <class Ut_>
  bool operator<(const Ut_ &rhs) {
    return value_ < rhs;
  }

  template <class Ut_>
  bool operator==(const Ut_ &rhs) {
    return rhs == value_;
  }

  template <class Ut_>
  bool operator!=(const Ut_ &rhs) {
    return rhs != value_;
  }

  template <class Ut_>
  void operator+=(const Ut_ &rhs) {
    value_ += rhs;
  }

  template <class Ut_>
  void operator++() {
    value_++;
  }

  template <class Ut_>
  void operator-=(const Ut_ &rhs) {
    value_ -= rhs;
  }

  template <class Ut_>
  void operator--() {
    value_--;
  }

  template <class Ut_>
  void operator*=(const Ut_ &rhs) {
    value_ *= rhs;
  }

  template <class Ut_>
  void operator/=(const Ut_ &rhs) {
    value_ /= rhs;
  }

  template <class Ut_>
  int operator+(const Ut_ &rhs) {
    return value_ + rhs;
  }

  template <class Ut_>
  int operator-(const Ut_ &rhs) {
    return value_ - rhs;
  }

  template <class Ut_>
  int operator*(const Ut_ &rhs) {
    return value_ * rhs;
  }

  template <class Ut_>
  int operator/(const Ut_ &rhs) {
    return value_ / rhs;
  }

  template <class Ut_>
  void operator=(const Ut_ &rhs) {
    SetValue(rhs);
  }

  Tp_ operator()() { return GetValue(); }

  //============================================================================
  // P U B L I C   M E T H O D S

  void SetValue(const Tp_ &value);

  const Tp_ &GetValue() const;

  void SetDescription(const std::string &description) override;

  std::string GetDescription() const override;

  void SetName(const std::string &name) override;

  std::string GetName() const override;

  virtual std::string ToString() const override;

  std::string GetType() const override;

  std::string GetStringValue() const override;

  void SetStringValue(const std::string &value) override;

 protected:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  std::string name_;

  Tp_ value_;

  std::string description_;
};

}  // namespace proc_image_processing

#include <proc_image_processing/server/parameter_inl.h>

#endif  // PROVIDER_VISION_FILTER_PARAMETER_H_
