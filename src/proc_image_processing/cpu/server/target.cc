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

#include <cpu/server/target.h>
namespace proc_image_processing {

//------------------------------------------------------------------------------
//
Target::Target()
    : center_(0, 0),
      dimension_(0, 0),
      angle_(0),
      header_(""),
      special_field_1_(""),
      special_field_2_("") {}

//------------------------------------------------------------------------------
//
Target::Target(const std::string &header, int x, int y, float width,
               float height, float angle, int image_height, int image_width,
               const std::string &spec_field_1,
               const std::string &spec_field_2) {
  SetTarget(header, x, y, width, height, angle, image_height, image_width,
            spec_field_1, spec_field_2);
}
}