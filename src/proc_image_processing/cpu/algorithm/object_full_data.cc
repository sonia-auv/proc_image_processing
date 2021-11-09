#include "object_full_data.h"

namespace proc_image_processing {

    ObjectFullData::ObjectFullData(const cv::Mat &originalImage, const cv::Mat &binaryImage, const Contour &contour)
            : ObjectBasicData(originalImage, binaryImage, contour) {
    }

    cv::Point objectVecMedian(const ObjectFullData::FullObjectPtrVec &objVec) {
        std::vector<float> xVec;
        std::vector<float> yVec;
        for (auto &elem : objVec) {
            xVec.insert(xVec.begin(), elem->getCenterPoint().x);
            yVec.insert(yVec.begin(), elem->getCenterPoint().y);
        }
        return {(int) (getMedian(xVec)), (int) (getMedian(yVec))};
    }

}  // namespace proc_image_processing
