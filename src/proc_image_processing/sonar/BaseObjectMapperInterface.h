/**
 * \file	BaseObjectMapperInterface.cc
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef PROC_IMAGE_PROCESSING_OBJECTMAPPER_H
#define PROC_IMAGE_PROCESSING_OBJECTMAPPER_H

#include "MapObject.h"

// Interface for all object mapper (vision, sonar, hydrophones, etc)
class BaseObjectMapperInterface {
public:
  virtual void GetMapObject(MapObjectVector &list) = 0;
  virtual void ResetMapper() = 0;
};


#endif //PROC_IMAGE_PROCESSING_OBJECTMAPPER_H
