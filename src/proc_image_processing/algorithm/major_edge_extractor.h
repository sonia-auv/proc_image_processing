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

  typedef ReferencePoint* RefPointPtr;
  typedef cv::Mat_<RefPointPtr> RefImage;

  class RefKernel {
  public:
    using Ptr = std::shared_ptr<RefKernel>;

    RefKernel(const RefPointPtr& north, const RefPointPtr& west,
      const RefPointPtr& center);

    ~RefKernel() {};
    RefPointPtr _north;
    RefPointPtr _west;
    RefPointPtr _center;
  };


  class MajorEdgeExtractor {
  public:
    using Ptr = std::shared_ptr<MajorEdgeExtractor>;

    static const float PERCENT_OF_VAL_FOR_VALUE_CONNECTION;

    cv::Mat ExtractEdge(const cv::Mat& image, int extreme_minimum);

  private:
    void Init(const cv::Size& size);

    void Clean();

    void CreateRefImage(const cv::Size& size);

    void AddRef(int x, int y, float value);

    bool IsAloneRef(const RefKernel& ref_kernel) const;

    bool IsNorthExist(const RefKernel& ref_kernel) const;

    bool IsWestExist(const RefKernel& ref_kernel) const;

    bool IsBothNortAndWestExist(const RefKernel& ref_kernel) const;

    bool IsValueConnected(const RefPointPtr ref, float value) const;

    bool IsValueGreater(const RefPointPtr ref, float value) const;

    bool IsJunction(const RefKernel& ref_kernel, float value) const;

    void SetLink(const RefPointPtr ref, float value, int x, int y);

    void SetJunction(RefKernel& ref_kernel, float value, int x, int y);

    float GetValInReferenceVec(int index);

    float GetValInReferenceVec(RefPointPtr ptr);

    void SetValInReferenceVec(int index, float value);

    void SetValInReferenceVec(RefPointPtr ptr, float value);


    RefImage ref_image_;

    std::vector<float> max_value_reference_;

    cv::Size img_size_;

    friend class MajorEdgeExtractorUT;
  };

  inline void MajorEdgeExtractor::CreateRefImage(const cv::Size& size) {
    ref_image_ = RefImage(size);
    // Free the RefPoint allocated previously.
    for (int y = 0, rows = ref_image_.rows, cols = ref_image_.cols; y < rows;
      y++) {
      RefPointPtr* ptr = ref_image_.ptr<RefPointPtr>(y);
      for (int x = 0; x < cols; x++) {
        ptr[x] = nullptr;
      }
    }
  }

  inline void MajorEdgeExtractor::AddRef(int x, int y, float value) {
    max_value_reference_.push_back(value);
    ref_image_.at<RefPointPtr>(y, x) =
      new ReferencePoint(value, max_value_reference_.size() - 1);
  }

  inline bool MajorEdgeExtractor::IsAloneRef(const RefKernel& ref_kernel) const {
    // if north or west exist, not alone
    return !(IsNorthExist(ref_kernel) || IsWestExist(ref_kernel));
  }

  inline bool MajorEdgeExtractor::IsNorthExist(
    const RefKernel& ref_kernel) const {
    return ref_kernel._north != nullptr;
  }

  inline bool MajorEdgeExtractor::IsWestExist(const RefKernel& ref_kernel) const {
    return ref_kernel._west != nullptr;
  }

  inline bool MajorEdgeExtractor::IsBothNortAndWestExist(
    const RefKernel& ref_kernel) const {
    return IsNorthExist(ref_kernel) && IsWestExist(ref_kernel);
  }

  inline bool MajorEdgeExtractor::IsValueConnected(const RefPointPtr ref,
    float value) const {
    if (ref != nullptr)
      return float(ref->_pix_value) * PERCENT_OF_VAL_FOR_VALUE_CONNECTION <=
      value;
    return false;
  }

  inline bool MajorEdgeExtractor::IsValueGreater(const RefPointPtr ref,
    float value) const {
    if (ref != nullptr)
      return max_value_reference_[ref->_reference_max_index] < value;
    return false;
  }

  inline bool MajorEdgeExtractor::IsJunction(const RefKernel& ref_kernel,
    float value) const {
    return IsValueConnected(ref_kernel._north, value) &&
      IsValueConnected(ref_kernel._west, value);
  }

  inline void MajorEdgeExtractor::SetLink(const RefPointPtr ref, float value,
    int x, int y) {
    if (ref != nullptr) {
      ref_image_.at<RefPointPtr>(y, x) =
        new ReferencePoint(value, ref->_reference_max_index);
      if (IsValueGreater(ref, value)) {
        SetValInReferenceVec(ref, value);
      }
    }
  }

  inline float MajorEdgeExtractor::GetValInReferenceVec(int index) {
    return max_value_reference_[index];
  }

  inline float MajorEdgeExtractor::GetValInReferenceVec(RefPointPtr ptr) {
    return GetValInReferenceVec(ptr->_reference_max_index);
  }

  inline void MajorEdgeExtractor::SetValInReferenceVec(int index, float value) {
    max_value_reference_[index] = value;
  }

  inline void MajorEdgeExtractor::SetValInReferenceVec(RefPointPtr ptr,
    float value) {
    SetValInReferenceVec(ptr->_reference_max_index, value);
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_MAJOR_EDGE_EXTRACTOR_H_
