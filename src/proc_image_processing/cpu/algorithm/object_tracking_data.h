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

#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_TRACKING_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_TRACKING_DATA_H_

#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/memory"

namespace proc_image_processing {

// OBjectTrackingData is a basic container class that holds information
// about an object's past state in time. EX. In the past X frames, how
// many time was the object present? Of How much the area or ratio changed
// in the past frames?
class ObjectTrackingData {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectTrackingData>;

  //============================================================================
  // P U B L I C   C / D T O R S

  ObjectTrackingData() : presence_count_(0.0f){};

  virtual ~ObjectTrackingData(){};

  //============================================================================
  // P U B L I C   M E T H O D S

  void SetPresenceCount(float presenceCount);

  float GetPresenceCount();

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  // In percent, nb of presence/ nb of frame in memory
  float presence_count_;

  // In percent, the variation of the ratio value
  // float _ratio_variation;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void ObjectTrackingData::SetPresenceCount(float presenceCount) {
  presence_count_ = presenceCount;
}

//------------------------------------------------------------------------------
//
inline float ObjectTrackingData::GetPresenceCount() { return presence_count_; }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_TRACKING_DATA_H_
