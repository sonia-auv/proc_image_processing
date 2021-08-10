#include "major_edge_extractor.h"

namespace proc_image_processing {

    const float MajorEdgeExtractor::PERCENT_OF_VAL_FOR_VALUE_CONNECTION = 0.8;

    ReferencePoint::ReferencePoint(float pix_val, int max_val_index)
            : _pix_value(pix_val), _reference_max_index(max_val_index) {
    }

    RefKernel::RefKernel(const RefPointPtr &north, const RefPointPtr &west,
                         const RefPointPtr &center)
            : _north(north), _west(west), _center(center) {
    }

    void MajorEdgeExtractor::init(const cv::Size &size) {
        // If the image is already created, no need for
        // re-creating it.
        if (ref_image_.size() != size) createReferenceImage(size);
    }

    void MajorEdgeExtractor::clean() {
        // Free the RefPoint allocated previously.
        max_value_reference_.clear();
        for (int y = 0, rows = ref_image_.rows, cols = ref_image_.cols; y < rows;
             y++) {
            auto *ptr = ref_image_.ptr<RefPointPtr>(y);
            for (int x = 0; x < cols; x++) {
                if (ptr[x] != nullptr) {
                    delete ptr[x];
                }
            }
        }
    }

    void MajorEdgeExtractor::setJunction(RefKernel &ref_kernel, float value, int x,
                                         int y) {
        if (ref_kernel._north == nullptr || ref_kernel._west == nullptr) return;

        RefPointPtr first_value = ref_kernel._north;
        RefPointPtr second_value = ref_kernel._west;

        if (getValueInReferenceVector(first_value) < getValueInReferenceVector(second_value)) {
            std::swap(first_value, second_value);
        }

        setLink(first_value, value, x, y);

        setValueInReferenceVector(second_value, getValueInReferenceVector(first_value));
    }

    cv::Mat MajorEdgeExtractor::extractEdge(
            const cv::Mat &image,
            int extreme_minimum
    ) {
        if (image.channels() != 1 || image.type() != CV_32F) {
            std::cout << "Bad image type or number of channel" << std::endl;
            return cv::Mat::zeros(1, 1, CV_8UC1);
        }

        // Image creation
        cv::Mat final_image(image.size(), CV_8UC1, nullptr);
        cv::Mat working_image;
        cv::copyMakeBorder(image, working_image, 1, 1, 1, 1, cv::BORDER_DEFAULT);
        init(working_image.size());

        for (int y = 1, rows = working_image.rows, cols = working_image.cols; y < rows - 1; y++) {
            auto *ptr = working_image.ptr<float>(y);
            auto *ref_up_line = ref_image_.ptr<RefPointPtr>(y - 1);
            auto *ref_center_line = ref_image_.ptr<RefPointPtr>(y);
            for (int x = 1; x < cols - 1; x++) {
                RefKernel ref_kernel(ref_up_line[x], ref_center_line[x - 1], ref_center_line[x]);
                float pix_val = ptr[x];
                // Pixel is too low in value, does not workt being looked at...
                if (pix_val < extreme_minimum) {
                    continue;
                }

                if (isAloneReference(ref_kernel)) {
                    addReferencePoint(x, y, pix_val);
                    continue;
                }

                if (isNorthAndWestExist(ref_kernel) && isJunction(ref_kernel, pix_val)) {
                    setJunction(ref_kernel, pix_val, x, y);
                    continue;
                }

                if (isWestExist(ref_kernel) && isValueConnected(ref_kernel._west, pix_val)) {
                    setLink(ref_kernel._west, pix_val, x, y);
                } else if (isNorthExist(ref_kernel) && isValueConnected(ref_kernel._north, pix_val)) {
                    setLink(ref_kernel._north, pix_val, x, y);
                }
            }
        }

        for (int y = 0, rows = final_image.rows, cols = final_image.cols; y < rows;
             y++) {
            auto *val_ptr = final_image.ptr<float>(y);
            auto *ref_ptr = ref_image_.ptr<RefPointPtr>(y + 1);

            for (int x = 0; x < cols; x++) {
                if (ref_ptr[x + 1] != nullptr) {
                    // val_ptr[x] = getValueInReferenceVector(ref_ptr[x+1]);
                    val_ptr[x] = 255;
                }
            }
        }

        clean();

        return final_image;
    }

}  // namespace proc_image_processing
