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

#ifndef PROVIDER_VISION_RANGED_PARAMETER_H_
#define PROVIDER_VISION_RANGED_PARAMETER_H_

#include <sonia_common/macros.h>
#include <proc_image_processing/server/parameter.h>
#include <iostream>
#include <string>
#include <vector>

namespace proc_image_processing {

template <typename Tp_>
class RangedParameter : public Parameter<Tp_> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RangedParameter<Tp_>>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit RangedParameter(const std::string &name, const Tp_ &value,
                           const Tp_ &min, const Tp_ &max,
                           std::vector<ParameterInterface *> *vector,
                           const std::string &description = "")
      : Parameter<Tp_>(name, value, vector, description),
        min_(min),
        max_(max) {}

  virtual ~RangedParameter() = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  const Tp_ &GetMin() const { return min_; }

  void SetMin(const Tp_ &min) { min_ = min; }

  const Tp_ &GetMax() const { return max_; }

  void SetMax(const Tp_ &max) { max_ = max; }

  std::string ToString() const override {
    std::stringstream ss;
    ss << Parameter<Tp_>::GetName() << Parameter<Tp_>::SEPARATOR;
    ss << Parameter<Tp_>::GetType() << Parameter<Tp_>::SEPARATOR;
    ss << Parameter<Tp_>::GetStringValue() << Parameter<Tp_>::SEPARATOR;
    ss << details::StringConvertor<Tp_>::GetString(min_)
       << Parameter<Tp_>::SEPARATOR;
    ss << details::StringConvertor<Tp_>::GetString(max_)
       << Parameter<Tp_>::SEPARATOR;
    ss << Parameter<Tp_>::GetDescription();
    return ss.str();
  }

 protected:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  Tp_ min_;

  Tp_ max_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_RANGED_PARAMETER_H_
