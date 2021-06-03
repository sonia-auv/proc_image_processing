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

#ifndef PROVIDER_VISION_ALGORITHM_TARGET_H_
#define PROVIDER_VISION_ALGORITHM_TARGET_H_

#include "../algorithm/general_function.h"
#include "../algorithm/object_full_data.h"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/stdlib.h"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/memory"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/local/include/opencv2/opencv.hpp"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/queue"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/home/sonia/base_lib_ws/devel/include/sonia_common/VisionTarget.h"

namespace proc_image_processing {

class Target {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Target>;

  //============================================================================
  // P U B L I C   C / D T O R S

  Target();
  Target(const std::string &header, int x, int y, float width, float height,
         float angle, int image_height, int image_width,
         const std::string &spec_field_1 = "",
         const std::string &spec_field_2 = "");

  ~Target(){};

  //============================================================================
  // P U B L I C   M E T H O D S

  // Setting target will use offseted center.
  void SetTarget(const std::string &header, int x, int y, float width,
                 float height, float angle, int image_height, int image_width,
                 const std::string &spec_field_1 = "",
                 const std::string &spec_field_2 = "");

  void SetTarget(ObjectFullData::Ptr obj, const std::string &header,
                 const std::string &spec_field_1 = "",
                 const std::string &spec_field_2 = "");

  void SetHeader(const std::string &header);

  void SetCenter(int x, int y);

  void SetCenter(const cv::Point &pt);

  void SetSize(int width, int height);

  void SetSize(const cv::Size &sz);

  void SetAngle(float angle);

  void SetSpecField_1(const std::string &field);

  void SetSpecField_2(const std::string &field);

  void SetSpecFields(const std::string &field1, const std::string &field2);

  std::string GetSpecField_1();

  std::string GetSpecField_2();

  void SetMessage(sonia_common::VisionTarget &msg);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S
  cv::Point center_;

  cv::Size_<float> dimension_;

  float angle_;

  // Bins name, buoy colors, etc.
  std::string header_;
  std::string special_field_1_;
  std::string special_field_2_;
};

typedef std::queue<Target> TargetQueue;

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void Target::SetTarget(const std::string &header, int x, int y,
                              float width, float height, float angle,
                              int image_height, int image_width,
                              const std::string &spec_field_1,
                              const std::string &spec_field_2) {
  header_ = header;
  center_.x = x;
  center_.y = y;
  dimension_.width = width;
  dimension_.height = height;
  angle_ = angle;
  special_field_1_ = spec_field_1;
  special_field_2_ = spec_field_2;
  SetCameraOffset(center_, image_height, image_width);
}

//------------------------------------------------------------------------------
//
inline void Target::SetTarget(ObjectFullData::Ptr obj,
                              const std::string &header,
                              const std::string &spec_field_1,
                              const std::string &spec_field_2) {
  header_ = header;
  center_ = obj->GetCenter();
  dimension_ = obj->GetRotatedRect().size;
  angle_ = obj->GetAngle();
  special_field_1_ = spec_field_1;
  special_field_2_ = spec_field_2;
  SetCameraOffset(center_, obj->GetImageSize().height,
                  obj->GetImageSize().width);
}

//------------------------------------------------------------------------------
//
inline void Target::SetHeader(const std::string &header) { header_ = header; }

//------------------------------------------------------------------------------
//
inline void Target::SetCenter(int x, int y) {
  center_.x = x;
  center_.y = y;
}

//------------------------------------------------------------------------------
//
inline void Target::SetCenter(const cv::Point &pt) { center_ = pt; }

//------------------------------------------------------------------------------
//
inline void Target::SetSize(int width, int height) {
  dimension_.width = width;
  dimension_.height = height;
}

//------------------------------------------------------------------------------
//
inline void Target::SetSize(const cv::Size &sz) { dimension_ = sz; }

//------------------------------------------------------------------------------
//
inline void Target::SetAngle(float angle) { angle_ = angle; }

//------------------------------------------------------------------------------
//
inline void Target::SetSpecField_1(const std::string &field) {
  special_field_1_ = field;
}

//------------------------------------------------------------------------------
//
inline void Target::SetSpecField_2(const std::string &field) {
  special_field_2_ = field;
}

//------------------------------------------------------------------------------
//
inline void Target::SetSpecFields(const std::string &field1,
                                  const std::string &field2) {
  special_field_1_ = field1;
  special_field_2_ = field2;
}

//------------------------------------------------------------------------------
//
inline std::string Target::GetSpecField_1() { return special_field_1_; }

//------------------------------------------------------------------------------
//
inline std::string Target::GetSpecField_2() { return special_field_2_; }

//------------------------------------------------------------------------------
//
inline void Target::SetMessage(sonia_common::VisionTarget &msg) {
  msg.header = header_;
  msg.x = center_.x;
  msg.y = center_.y;
  msg.width = dimension_.width;
  msg.height = dimension_.height;
  msg.angle = angle_;
  msg.desc_1 = special_field_1_;
  msg.desc_2 = special_field_2_;
}

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_TARGET_H_
