/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FRAME_MEMORY_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FRAME_MEMORY_H_

#include "general_function.h"
#include "object_full_data.h"
#include <memory>
#include <vector>

namespace proc_image_processing {

    class ObjectFrameMemory {
    public:
        using Ptr = std::shared_ptr<ObjectFrameMemory>;

        // When getting object in the past, we compare the center
        // and the ratio to make sure it stills fit the same object.
        // If the ratio difference is smaller than RATIO_MAX_DIFFERENCE
        // we consider it as good object.
        static const float DISTANCE_MAX_DIFFERENCE;
        static const float RATIO_MAX_DIFFERENCE;

        ObjectFrameMemory(unsigned int memorySize);

        ~ObjectFrameMemory() {}

        void AddFrameObjects(ObjectFullData::FullObjectPtrVec &objectVector);

        unsigned int GetMemorySize();

        // Use the center and the ratio to find an object in the past object list.
        ObjectFullData::FullObjectPtrVec GetPastObjectsViaCenter(
                const cv::Point &center, const float objectRatio);

    private:
        std::vector<ObjectFullData::FullObjectPtrVec> previous_frames_;
        unsigned int memory_size_;
    };

    inline unsigned int ObjectFrameMemory::GetMemorySize() { return memory_size_; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FRAME_MEMORY_H_
