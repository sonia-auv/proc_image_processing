/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_TARGET_H_
#define PROVIDER_VISION_ALGORITHM_TARGET_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include <stdlib.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <sonia_common/VisionTarget.h>

namespace proc_image_processing {

  class Target {
  public:


    using Ptr = std::shared_ptr<Target>;



    Target();
    Target(const std::string& header, int x, int y, float width, float height,
      float angle, int image_height, int image_width,
      const std::string& spec_field_1 = "",
      const std::string& spec_field_2 = "");

      ~Target() {};


      // Setting target will use offseted center.
      void setTarget(const std::string &header, int x, int y, float width,
                     float height, float angle, int image_height, int image_width,
                     const std::string &spec_field_1 = "",
                     const std::string &spec_field_2 = "");

      void setTarget(ObjectFullData::Ptr obj, const std::string &header,
                     const std::string &spec_field_1 = "",
                     const std::string &spec_field_2 = "");

      void setHeader(const std::string &header);

      void setCenter(int x, int y);

      void setCenter(const cv::Point &pt);

      void setSize(int width, int height);

      void setSize(const cv::Size &sz);

      void setAngle(float angle);

      void setSpecField1(const std::string &field);

      void setSpecField2(const std::string &field);

      void setSpecFields(const std::string &field1, const std::string &field2);

      std::string getSpecField1();

      std::string getSpecField2();

      void setMessage(sonia_common::VisionTarget &msg);

  private:

    cv::Point center_;

    cv::Size_<float> dimension_;

    float angle_;

      // Bins name, buoy colors, etc.
      std::string header_;
      std::string special_field_1_;
      std::string special_field_2_;
  };

    typedef std::queue<Target> TargetQueue;


    inline void Target::setTarget(const std::string &header, int x, int y,
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
        setCameraOffset(center_, image_height, image_width);
    }

    inline void Target::setTarget(ObjectFullData::Ptr obj,
                                  const std::string &header,
                                  const std::string &spec_field_1,
                                  const std::string &spec_field_2) {
        header_ = header;
        center_ = obj->getCenterPoint();
        dimension_ = obj->getRotRect().size;
        angle_ = obj->getAngle();
        special_field_1_ = spec_field_1;
        special_field_2_ = spec_field_2;
        setCameraOffset(center_, obj->getImageSize().height,
                        obj->getImageSize().width);
    }

    inline void Target::setHeader(const std::string &header) { header_ = header; }

    inline void Target::setCenter(int x, int y) {
        center_.x = x;
        center_.y = y;
    }

    inline void Target::setCenter(const cv::Point &pt) { center_ = pt; }

    inline void Target::setSize(int width, int height) {
        dimension_.width = width;
        dimension_.height = height;
    }

    inline void Target::setSize(const cv::Size &sz) { dimension_ = sz; }

    inline void Target::setAngle(float angle) { angle_ = angle; }

    inline void Target::setSpecField1(const std::string &field) {
        special_field_1_ = field;
    }

    inline void Target::setSpecField2(const std::string &field) {
        special_field_2_ = field;
    }

    inline void Target::setSpecFields(const std::string &field1,
                                      const std::string &field2) {
        special_field_1_ = field1;
        special_field_2_ = field2;
    }

    inline std::string Target::getSpecField1() { return special_field_1_; }

    inline std::string Target::getSpecField2() { return special_field_2_; }

    inline void Target::setMessage(sonia_common::VisionTarget &msg) {
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
