/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "object_full_data.h"

namespace proc_image_processing {

    ObjectFullData::ObjectFullData(const cv::Mat &originalImage, const cv::Mat &binaryImage, const Contour &contour)
            : ObjectBasicData(originalImage, binaryImage, contour) {
    }

    [[maybe_unused]] cv::Point objectVecMedian(const ObjectFullData::FullObjectPtrVec &objVec) {
        std::vector<float> xVec;
        std::vector<float> yVec;
        for (auto &elem : objVec) {
            xVec.insert(xVec.begin(), elem->getCenterPoint().x);
            yVec.insert(yVec.begin(), elem->getCenterPoint().y);
        }
        return cv::Point((int) (getMedian(xVec)),
                         (int) (getMedian(yVec)));
    }

}  // namespace proc_image_processing
