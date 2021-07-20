#include "object_frame_memory.h"

namespace proc_image_processing {

    // 20 pix offset center should cover small noise error and small displacement.
    const float ObjectFrameMemory::DISTANCE_MAX_DIFFERENCE = 20;

    // A difference of 10% of ratio is big enough to discard an object.
    const float ObjectFrameMemory::RATIO_MAX_DIFFERENCE = 0.1;

    ObjectFrameMemory::ObjectFrameMemory(unsigned int memorySize)
            : previous_frames_(memorySize), memory_size_(memorySize) {
    }

    void ObjectFrameMemory::AddFrameObjects(
            ObjectFullData::FullObjectPtrVec &objectVector) {
        previous_frames_.push_back(objectVector);
    }

    ObjectFullData::FullObjectPtrVec ObjectFrameMemory::GetPastObjectsViaCenter(
            const cv::Point &center, const float objectRatio) {
        ObjectFullData::FullObjectPtrVec objVec;

        // For i frame
        for (int i = 0, buffSize = previous_frames_.size(); i < buffSize; i++) {
            float shortestDistance = 100000.0f;
            ObjectFullData::Ptr nearestObject = nullptr;
            ObjectFullData::FullObjectPtrVec currentFrameData = previous_frames_.at(i);

            // for all the object in the frame
            for (int j = 0, size = currentFrameData.size(); j < size; j++) {
                ObjectFullData::Ptr analysedObject = currentFrameData[j];
                if (analysedObject.get() != nullptr) {
                    cv::Point analysedCenter = analysedObject->getCenterPoint();
                    float distance = getEuclideanDistance(center, analysedCenter);

                    float analysedRatio = analysedObject->getRatio();
                    float ratioDifference = fabsf(analysedRatio - objectRatio);

                    if (distance < shortestDistance && distance < DISTANCE_MAX_DIFFERENCE &&
                        ratioDifference < RATIO_MAX_DIFFERENCE) {
                        shortestDistance = distance;
                        nearestObject = analysedObject;
                    }
                }
            }
            if (nearestObject.get() != nullptr) {
                objVec.push_back(nearestObject);
            }
        }
        return objVec;
    }

}  // namespace proc_image_processing
