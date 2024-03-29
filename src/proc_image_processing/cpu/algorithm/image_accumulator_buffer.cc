#include "image_accumulator_buffer.h"

namespace proc_image_processing {

    ImageAccumulatorBuffer::ImageAccumulatorBuffer(int bufferLength, const cv::Size &imgSize, int type, METHOD method)
            : buffer_size_(bufferLength),
              individual_weight_(0.0f),
              buffer_current_index_(0),
              image_vec_(0),
              image_type_(type),
              image_size_(imgSize),
              average_method_(nullptr) {
        // Start with a buffer filled with blank matrices.
        fillWithBlankImages();
        individual_weight_ = 1.0 / static_cast<float>(bufferLength);
        setAverageMethod(method);
    }

    void ImageAccumulatorBuffer::convertImage(cv::Mat &image) {
        // We do not want to access blank...&
        if (buffer_size_ == 0) return;
        // Use the function in memory to average the accumulator and
        // reset the values
        (this->*average_method_)(image);
        image.convertTo(image, image_type_);
    }

    void ImageAccumulatorBuffer::addImage(const cv::Mat &image) {
        if (image_size_ != image.size() || image_type_ != image.type()) {
            printf(
                    "ImageAccumulatorBuffer: Image type or image size is different from "
                    "the "
                    "one I was constructed with.\n");
            return;
        }

        // Put everything in float, so we don't have issues with decimals and
        // image type mismatch
        // CV_MAT_CN return the number of channel.
        int index = buffer_current_index_ % buffer_size_;
        image.convertTo(image_vec_[index], CV_32FC(CV_MAT_CN(image_type_)));
        // Here, since unsigned value, will return to zero after "overflowing"
        buffer_current_index_++;
    }

    void ImageAccumulatorBuffer::resetBuffer() { fillWithBlankImages(); }

    void ImageAccumulatorBuffer::resetBuffer(int bufferLength, const cv::Size &imgSize, int type) {
        buffer_size_ = bufferLength;
        image_size_ = imgSize;
        image_type_ = type;
        buffer_current_index_ = 0;
        individual_weight_ = 1.0 / static_cast<float>(bufferLength);

        fillWithBlankImages();
    }

    void ImageAccumulatorBuffer::averageUsingSameWeights(cv::Mat &resultImage) {
        if (buffer_size_ == 0) {
            printf("Image accumulator size is 0\n");
            return;
        }

        cv::Mat adderImage =
                cv::Mat::zeros(image_size_, CV_32FC(CV_MAT_CN(image_type_)));
        for (size_t i = 0; i < buffer_size_; i++) {
            cv::add(adderImage, image_vec_[i], adderImage);
        }
        // Divide and cast into the image type given in at construction or resizing.
        cv::divide(adderImage, cv::Mat(image_size_, CV_32FC(CV_MAT_CN(image_type_)),
                                       cv::Scalar::all(buffer_size_)),
                   resultImage);
    }

    void ImageAccumulatorBuffer::averageUsingNewestAsPriority(cv::Mat &resultImage) {
        resultImage = image_vec_[getIndexRelativeToOldest(0)].clone();
        for (size_t i = 1; i < buffer_size_; i++) {
            cv::addWeighted(resultImage, 0.5, image_vec_[getIndexRelativeToOldest(i)], 0.5, 0,
                            resultImage);
        }
    }

    void ImageAccumulatorBuffer::averageUsingDecreasingWeights(
            cv::Mat &resultImage) {
        // Fill resultImage
        // Keep in float to stay at full scale.
        resultImage = cv::Mat::zeros(image_size_, CV_32FC(CV_MAT_CN(image_type_)));
        cv::addWeighted(resultImage, 0.5, image_vec_[getIndexRelativeToNewest(0)], 0.5,
                        0, resultImage);

        float resultingWeight = 0.50f;
        // Start at one, we already did the 0 index (newest)
        for (size_t i = 1; i < buffer_size_; i++) {
            // Addjust the weight, we are one element older now then before,
            // so the weight of this element is half the precendent.
            resultingWeight = resultingWeight / 2;

            cv::add(resultImage,
                    image_vec_[getIndexRelativeToNewest(i)] * resultingWeight,
                    resultImage);
        }
    }

}  // namespace proc_image_processing
