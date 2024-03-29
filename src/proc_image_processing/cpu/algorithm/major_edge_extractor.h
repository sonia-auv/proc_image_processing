#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_

#include <memory>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

    class ReferencePoint {
    public:
        using Ptr = std::shared_ptr<ReferencePoint>;

        ReferencePoint(float pix_val, int max_val_index);

        float _pix_value;
        int _reference_max_index;
    };

    typedef ReferencePoint *RefPointPtr;
    typedef cv::Mat_<RefPointPtr> RefImage;

    class RefKernel {
    public:
        using Ptr = std::shared_ptr<RefKernel>;

        RefKernel(const RefPointPtr &north, const RefPointPtr &west,
                  const RefPointPtr &center);

        ~RefKernel() = default;

        RefPointPtr _north;
        RefPointPtr _west;
        RefPointPtr _center;
    };


    class MajorEdgeExtractor {
    public:
        using Ptr = std::shared_ptr<MajorEdgeExtractor>;

        static const float PERCENT_OF_VAL_FOR_VALUE_CONNECTION;

        explicit MajorEdgeExtractor(const RefImage &refImage);

        cv::Mat extractEdge(const cv::Mat &image, int extreme_minimum);

    private:
        void init(const cv::Size &size);

        void clean();

        void createReferenceImage(const cv::Size &size);

        // TODO is it really adding or it is just setting the reference point?!
        void addReferencePoint(int x, int y, float value);

        static bool isAloneReference(const RefKernel &ref_kernel);

        static bool isNorthExist(const RefKernel &ref_kernel);

        static bool isWestExist(const RefKernel &ref_kernel);

        static bool isNorthAndWestExist(const RefKernel &ref_kernel);

        static bool isValueConnected(RefPointPtr ref, float value);

        bool isValueGreater(RefPointPtr ref, float value) const;

        static bool isJunction(const RefKernel &ref_kernel, float value);

        float getValueInReferenceVector(int index);

        float getValueInReferenceVector(RefPointPtr ptr);

        void setLink(RefPointPtr ref, float value, int x, int y);

        void setJunction(RefKernel &ref_kernel, float value, int x, int y);

        void setValueInReferenceVector(int index, float value);

        void setValueInReferenceVector(RefPointPtr ptr, float value);


        RefImage ref_image_;

        std::vector<float> max_value_reference_;

        cv::Size img_size_;

        friend class MajorEdgeExtractorUT;
    };

    inline void MajorEdgeExtractor::createReferenceImage(const cv::Size &size) {
        ref_image_ = RefImage(size);
        // Free the RefPoint allocated previously.
        for (int y = 0, rows = ref_image_.rows, cols = ref_image_.cols; y < rows;
             y++) {
            auto *ptr = ref_image_.ptr<RefPointPtr>(y);
            for (int x = 0; x < cols; x++) {
                ptr[x] = nullptr;
            }
        }
    }

    inline void MajorEdgeExtractor::addReferencePoint(int x, int y, float value) {
        max_value_reference_.push_back(value);
        ref_image_.at<RefPointPtr>(y, x) =
                new ReferencePoint(value, max_value_reference_.size() - 1);
    }

    inline bool MajorEdgeExtractor::isAloneReference(const RefKernel &ref_kernel) {
        // if north or west exist, not alone
        return !(isNorthExist(ref_kernel) || isWestExist(ref_kernel));
    }

    inline bool MajorEdgeExtractor::isNorthExist(const RefKernel &ref_kernel) {
        return ref_kernel._north != nullptr;
    }

    inline bool MajorEdgeExtractor::isWestExist(const RefKernel &ref_kernel) {
        return ref_kernel._west != nullptr;
    }

    inline bool MajorEdgeExtractor::isNorthAndWestExist(const RefKernel &ref_kernel) {
        return isNorthExist(ref_kernel) && isWestExist(ref_kernel);
    }

    inline bool MajorEdgeExtractor::isValueConnected(RefPointPtr ref, float value) {
        if (ref != nullptr)
            return ref->_pix_value * PERCENT_OF_VAL_FOR_VALUE_CONNECTION <= value;
        return false;
    }

    inline bool MajorEdgeExtractor::isValueGreater(RefPointPtr ref, float value) const {
        if (ref != nullptr)
            return max_value_reference_[ref->_reference_max_index] < value;
        return false;
    }

    inline bool MajorEdgeExtractor::isJunction(const RefKernel &ref_kernel,
                                               float value) {
        return isValueConnected(ref_kernel._north, value) &&
               isValueConnected(ref_kernel._west, value);
    }

    inline void MajorEdgeExtractor::setLink(RefPointPtr ref, float value, int x, int y) {
        if (ref != nullptr) {
            ref_image_.at<RefPointPtr>(y, x) =
                    new ReferencePoint(value, ref->_reference_max_index);
            if (isValueGreater(ref, value)) {
                setValueInReferenceVector(ref, value);
            }
        }
    }

    inline float MajorEdgeExtractor::getValueInReferenceVector(int index) {
        return max_value_reference_[index];
    }

    inline float MajorEdgeExtractor::getValueInReferenceVector(RefPointPtr ptr) {
        return getValueInReferenceVector(ptr->_reference_max_index);
    }

    inline void MajorEdgeExtractor::setValueInReferenceVector(int index, float value) {
        max_value_reference_[index] = value;
    }

    inline void MajorEdgeExtractor::setValueInReferenceVector(RefPointPtr ptr, float value) {
        setValueInReferenceVector(ptr->_reference_max_index, value);
    }

    MajorEdgeExtractor::MajorEdgeExtractor(const RefImage &refImage) : ref_image_(refImage) {}

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_
