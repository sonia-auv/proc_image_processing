/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_
#define PROVIDER_VISION_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_

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

        ~RefKernel() {};
        RefPointPtr _north;
        RefPointPtr _west;
        RefPointPtr _center;
    };


    class MajorEdgeExtractor {
    public:
        using Ptr = std::shared_ptr<MajorEdgeExtractor>;

        static const float PERCENT_OF_VAL_FOR_VALUE_CONNECTION;

        cv::Mat ExtractEdge(const cv::Mat &image, int extreme_minimum);

    private:
        void init(const cv::Size &size);

        void clean();

        void createReferenceImage(const cv::Size &size);

        // TODO is it really adding or it is just setting the reference point?!
        void addReferencePoint(int x, int y, float value);

        bool isAloneReference(const RefKernel &ref_kernel) const;

        bool isNorthExist(const RefKernel &ref_kernel) const;

        bool isWestExist(const RefKernel &ref_kernel) const;

        bool isNorthAndWestExist(const RefKernel &ref_kernel) const;

        bool isValueConnected(const RefPointPtr ref, float value) const;

        bool isValueGreater(const RefPointPtr ref, float value) const;

        bool isJunction(const RefKernel &ref_kernel, float value) const;

        float getValueInReferenceVector(int index);

        float getValueInReferenceVector(RefPointPtr ptr);

        void setLink(const RefPointPtr ref, float value, int x, int y);

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
            RefPointPtr *ptr = ref_image_.ptr<RefPointPtr>(y);
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

    inline bool MajorEdgeExtractor::isAloneReference(const RefKernel &ref_kernel) const {
        // if north or west exist, not alone
        return !(isNorthExist(ref_kernel) || isWestExist(ref_kernel));
    }

    inline bool MajorEdgeExtractor::isNorthExist(
            const RefKernel &ref_kernel) const {
        return ref_kernel._north != nullptr;
    }

    inline bool MajorEdgeExtractor::isWestExist(const RefKernel &ref_kernel) const {
        return ref_kernel._west != nullptr;
    }

    inline bool MajorEdgeExtractor::isNorthAndWestExist(
            const RefKernel &ref_kernel) const {
        return isNorthExist(ref_kernel) && isWestExist(ref_kernel);
    }

    inline bool MajorEdgeExtractor::isValueConnected(const RefPointPtr ref,
                                                     float value) const {
        if (ref != nullptr)
            return float(ref->_pix_value) * PERCENT_OF_VAL_FOR_VALUE_CONNECTION <=
                   value;
        return false;
    }

    inline bool MajorEdgeExtractor::isValueGreater(const RefPointPtr ref,
                                                   float value) const {
        if (ref != nullptr)
            return max_value_reference_[ref->_reference_max_index] < value;
        return false;
    }

    inline bool MajorEdgeExtractor::isJunction(const RefKernel &ref_kernel,
                                               float value) const {
        return isValueConnected(ref_kernel._north, value) &&
               isValueConnected(ref_kernel._west, value);
    }

    inline void MajorEdgeExtractor::setLink(const RefPointPtr ref, float value,
                                            int x, int y) {
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

    inline void MajorEdgeExtractor::setValueInReferenceVector(RefPointPtr ptr,
                                                              float value) {
        setValueInReferenceVector(ptr->_reference_max_index, value);
    }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_
