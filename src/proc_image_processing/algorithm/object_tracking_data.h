/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_TRACKING_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_TRACKING_DATA_H_

#include <memory>

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

    ObjectTrackingData() : presence_count_(0.0f) {};

    virtual ~ObjectTrackingData() {};

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
