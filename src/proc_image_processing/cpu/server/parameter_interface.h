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
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ParameterInterface>;

  //============================================================================
  // P U B L I C   C / D T O R S

  ParameterInterface() = default;

  virtual ~ParameterInterface() = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void SetDescription(const std::string &description) = 0;

  virtual std::string GetDescription() const = 0;

  virtual void SetName(const std::string &name) = 0;

  virtual std::string GetName() const = 0;

  virtual std::string ToString() const = 0;

  virtual std::string GetType() const = 0;

  virtual std::string GetStringValue() const = 0;

  virtual void SetStringValue(const std::string &) = 0;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_PARAMETER_INTERFACE_H_
