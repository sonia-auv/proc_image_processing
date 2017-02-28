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
#error This file may only be included from parameter.h
#endif

#include <cxxabi.h>
#include <lib_atlas/macros.h>
#include <opencv/cv.h>
#include <cstdlib>
#include <string>
#include <vector>

namespace proc_image_processing {

namespace details {

//------------------------------------------------------------------------------
//
template <typename Tp_>
struct StringConvertor {
  static std::string TypeName() {
    int status;
    std::string tname = typeid(Tp_).name();
    char *demangled_name =
        abi::__cxa_demangle(tname.c_str(), NULL, NULL, &status);
    if (status == 0) {
      tname = demangled_name;
      std::free(demangled_name);
    }
    return tname;
  }

  static std::string GetString(const Tp_ &value) {
    return std::to_string(value);
  }

  static Tp_ GetValue(const std::string &value) {
    return static_cast<Tp_>(value);
  }
};

//------------------------------------------------------------------------------
//
template <>
struct StringConvertor<int> {
  static std::string TypeName() { return "Integer"; }

  static std::string GetString(const int &value) {
    return std::to_string(value);
  }

  static int GetValue(const std::string &value) { return atoi(value.c_str()); }
};

//------------------------------------------------------------------------------
//
template <>
struct StringConvertor<bool> {
  static std::string TypeName() { return "Boolean"; }

  static std::string GetString(const bool &value) {
    if (value) {
      return "1";
    } else {
      return "0";
    }
  }

  static bool GetValue(const std::string &value) {
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

//------------------------------------------------------------------------------
//
template <>
struct StringConvertor<double> {
  static std::string TypeName() { return "Double"; }

  static std::string GetString(const double &value) {
    return std::to_string(value);
  }

  static double GetValue(const std::string &value) {
    return atof(value.c_str());
  }
};

//------------------------------------------------------------------------------
//
template <>
struct StringConvertor<std::string> {
  static std::string TypeName() { return "String"; }

  static std::string GetString(const std::string &value) { return value; }

  static std::string GetValue(const std::string &value) { return value; }
};

}  // namespace details

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE Parameter<Tp_>::Parameter(
    const std::string &name, const Tp_ &value,
    std::vector<ParameterInterface *> *vector, const std::string &description)
    : name_(name), value_(value), description_(description) {
  if (vector != nullptr) {
    vector->push_back(dynamic_cast<ParameterInterface *>(this));
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetDescription(
    const std::string &description) {
  description_ = description;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetDescription() const {
  return description_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetName(const std::string &name) {
  name_ = name;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetName() const {
  return name_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::ToString() const {
  std::stringstream ss;
  ss << GetName() << SEPARATOR;
  ss << GetType() << SEPARATOR;
  ss << GetStringValue() << SEPARATOR;
  ss << SEPARATOR << SEPARATOR;
  ss << GetDescription();
  return ss.str();
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetValue(const Tp_ &value) {
  value_ = value;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE const Tp_ &Parameter<Tp_>::GetValue() const {
  return value_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetType() const {
  return details::StringConvertor<Tp_>::TypeName();
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetStringValue() const {
  return details::StringConvertor<Tp_>::GetString(value_);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetStringValue(const std::string &value) {
  value_ = details::StringConvertor<Tp_>::GetValue(value);
}

}  // namespace proc_image_processing
