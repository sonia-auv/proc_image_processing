/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_TRACKING_DATA_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_TRACKING_DATA_H_

#include <memory>

namespace proc_image_processing {

    // ObjectTrackingData is a basic container class that holds information
    // about an object's past state in time. EX. In the past X frames, how
    // many time was the object present? Of How much the area or ratio changed
    // in the past frames?
    class ObjectTrackingData {
    public:
        using Ptr = std::shared_ptr<ObjectTrackingData>;

        ObjectTrackingData() : presence_count_(0.0f) {};

        virtual ~ObjectTrackingData() = default;

        void setPresenceCount(float presenceCount);

        float getPresenceCount() const;

    private:
        // In percent, nb of presence/ nb of frame in memory
        float presence_count_;

        // In percent, the variation of the ratio value
        // float _ratio_variation;
    };

    inline void ObjectTrackingData::setPresenceCount(float presenceCount) {
        presence_count_ = presenceCount;
    }

    inline float ObjectTrackingData::getPresenceCount() const { return presence_count_; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_TRACKING_DATA_H_
