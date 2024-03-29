#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FULL_DATA_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FULL_DATA_H_

#include "object_basic_data.h"
#include "object_ranking_data.h"
#include "object_tracking_data.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include "object_feature.h"
#include "general_function.h"

namespace proc_image_processing {

    // Simple container class that is created with the contour.
    // It inherits from the different caracteristic of an object
    // (in the time domain, as a contour and compared to others)
    // It is important to note that it does not calculate all the
    // basic characteristic, it waits until it is ask from the object
    // to calculated. That way, we do not waste calculation time for information
    // we wont use.
    // Also, for tracking and ranking data, it is necessary
    // to fill the object with the help of ObjectRanker and FrameMemory.
    class ObjectFullData : public ObjectTrackingData,
                           public ObjectBasicData,
                           public ObjectRankingData,
                           public ObjectFeatureData {
    public:
        using Ptr = std::shared_ptr<ObjectFullData>;

        typedef std::vector<ObjectFullData::Ptr> FullObjectPtrVec;

        ObjectFullData(const cv::Mat &originalImage, const cv::Mat &binaryImage, const Contour &contour);

        ~ObjectFullData() override = default;
    };

    inline bool areaSorts(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) {
        return a->getArea() < b->getArea();
    }

    inline bool ratioSorts(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) {
        return a->getRatio() < b->getRatio();
    }

    cv::Point objectVecMedian(const ObjectFullData::FullObjectPtrVec &);

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FULL_DATA_H_
