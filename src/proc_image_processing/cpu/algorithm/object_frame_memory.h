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

#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_FRAME_MEMORY_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_FRAME_MEMORY_H_

#include "general_function.h"
#include "object_full_data.h"
#include <memory>
#include <vector>

namespace proc_image_processing {

class ObjectFrameMemory {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectFrameMemory>;

  // When getting object in the past, we compare the center
  // and the ratio to make sure it stills fit the same object.
  // If the ratio difference is smaller than RATIO_MAX_DIFFERENCE
  // we consider it as good object.
  static const float DISTANCE_MAX_DIFFERENCE;
  static const float RATIO_MAX_DIFFERENCE;

  //============================================================================
  // P U B L I C   C / D T O R S

  ObjectFrameMemory(unsigned int memorySize);

  ~ObjectFrameMemory() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  void AddFrameObjects(ObjectFullData::FullObjectPtrVec &objectVector);

  unsigned int GetMemorySize();

  // Use the center and the ratio to find an object in the past object list.
  ObjectFullData::FullObjectPtrVec GetPastObjectsViaCenter(
      const cv::Point &center, const float objectRatio);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  std::vector<ObjectFullData::FullObjectPtrVec> previous_frames_;
  unsigned int memory_size_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline unsigned int ObjectFrameMemory::GetMemorySize() { return memory_size_; }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_FRAME_MEMORY_H_
