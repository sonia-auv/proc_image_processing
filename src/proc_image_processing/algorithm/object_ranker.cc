/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
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

#include <proc_image_processing/algorithm/object_ranker.h>

namespace proc_image_processing {

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void ObjectRanker::RankByArea(ObjectFullData::FullObjectPtrVec objects) {
  std::sort(objects.begin(), objects.end(), ObjectRanker::AreaSortFunction);
  for (int i = 0, size = objects.size(); i < size; i++) {
    if (objects[i].get() != nullptr) {
      objects[i]->SetAreaRank((float(size - i)) / float(size));
    }
  }
}

//------------------------------------------------------------------------------
//
void ObjectRanker::RankByLength(ObjectFullData::FullObjectPtrVec objects) {
  std::sort(objects.begin(), objects.end(), ObjectRanker::LengthSortFunction);
  for (int i = 0, size = objects.size(); i < size; i++) {
    if (objects[i].get() != nullptr) {
      objects[i]->SetLengthRank(float((size - i)) / float(size));
    }
  }
}

}  // namespace proc_image_processing
