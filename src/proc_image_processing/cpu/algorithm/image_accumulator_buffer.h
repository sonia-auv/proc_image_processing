/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_IMAGE_ACCUMULATOR_BUFFER_H_
#define PROVIDER_VISION_ALGORITHM_IMAGE_ACCUMULATOR_BUFFER_H_

#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

  /**
   * Simple circular buffer that returns the weighted sum of all images in his
   * buffer.
   */
  class ImageAccumulatorBuffer {
  public:
    using Ptr = std::shared_ptr<ImageAccumulatorBuffer>;

    enum METHOD { ACC_ALL_SAME_WEIGHT, ACC_50_PERCENT, ACC_ADJUST_WEIGHT };

    /**
     * Creates a circular buffer of bufferLength,
     * Filled with images of imgSize, with type type (8cu1, etc.).
     * At construction they are filled by zero, and they are replaced as the
     * images are added.
     *
     * NO CHECK IS MADE ON IMGSIZE AND TYPE, MAKE SURE YOU GIVE
     * TO THE FUNCTION PARAMETERS THAT WORK
     */
    ImageAccumulatorBuffer(int bufferLength, cv::Size imgSize, int type,
      METHOD method = ACC_ALL_SAME_WEIGHT);

    ~ImageAccumulatorBuffer() {};

    void addImage(const cv::Mat &image);

      void convertImage(cv::Mat &image);

      /**
       * No concurrency access check on buffer is made,  Do not use in different
       * thread.
       */
      void resetBuffer();

      /**
       * No check is made on bufferLength.
       * Resize the accumulator and reset it.
       */
      void resetBuffer(int bufferLength, cv::Size imgSize, int type);

      /**
       * Set the average method via the Enum.
       */
      void setAverageMethod(METHOD method);

      /**
       * Return the buffer length
       */
      int getBufferLength();

      /**
       * Usefull for for loop iteration. ZERO BASED
       * Return the index of the element at elementNumber from the most
       * recent.
       *
       * If the newest is at idx 4 (on a array of size 5)
       * elementNumber == 0 will return 4, and
       * elementNumber == 1 will return 3, etc.
       */
      int getIndexRelativeToNewest(int elementNumber);

      /**
       * Return the index of the element at elementNumber from the oldest.
       *
       * If the oldest is at idx 4 (on a array of size 5)
       * elementNumber == 0 will return 4, and
       * elementNumber == 1 will return 0, etc.
       */
      int getIndexRelativeToOldest(int elementNumber);

      void getImage(size_t index, cv::Mat &image);

  private:
      /**
       * Averaging methods
       * They all keep the image in CV_32FCN
       * All images in the buffer have the same weights
       */
      void averageUsingSameWeights(cv::Mat &resultImage);

      /**
       * Newest image has the most importance.
       *
       * for size of 4,Img1 is older image, Img4 is newest
       * result = Img1*0.50 + Img2* 0.50
       * result = result*0.50 + Img3 * 0.50
       * result = result*0.50 + Img4 * 0.50
       */
      void averageUsingNewestAsPriority(cv::Mat &resultImage);

      /**
       * for size of 4,Img1 is older image, Img4 is newest
       * result = Img4*0.50 + Img3*0.25 + Img2*0.125 + Imag*0.0625
       */
      void averageUsingDecreasingWeights(cv::Mat &resultImage);

      /**
       * Fills the accumulator with blank images.
       */
      void fillWithBlankImages();


    size_t buffer_size_;

    double individual_weight_;

    uint buffer_current_index_;

    std::vector<cv::Mat> image_vec_;

    int image_type_;

    cv::Size image_size_;

    /**
     * Pointer to the method to use for averaging the frame
     */
    void (ImageAccumulatorBuffer::* average_method_)(cv::Mat&);
  };

    inline int ImageAccumulatorBuffer::getBufferLength() { return buffer_size_; }

    inline void ImageAccumulatorBuffer::getImage(size_t index, cv::Mat &image) {
        if (index < buffer_size_) {
            image_vec_[index].copyTo(image);
        }
    }

    inline int ImageAccumulatorBuffer::getIndexRelativeToNewest(int elementNumber) {
        // Newest frame
        int index = buffer_current_index_ % buffer_size_;
        index -= elementNumber;
        // return at the end of the vector to continue moving
        if (index < 0) {
            // In reality its _buffer_size - abs(index) but y'know
            // computation and stuff
            index = buffer_size_ + index;
        }
        return index;
  }

    inline int ImageAccumulatorBuffer::getIndexRelativeToOldest(int elementNumber) {
        // Newest frame
        int index = buffer_current_index_ % buffer_size_;
        return (index + 1 + elementNumber) % buffer_size_;
    }

    inline void ImageAccumulatorBuffer::fillWithBlankImages() {
        cv::Mat zero = cv::Mat::zeros(image_size_, image_type_);
        // It is possible to call fillWithBlank when the buffer is active,
        // we must clear it before pushing back new values.
        image_vec_.clear();
        for (size_t i = 0; i < buffer_size_; i++) {
            image_vec_.push_back(zero);
        }
    }

    inline void ImageAccumulatorBuffer::setAverageMethod(METHOD method) {
        switch (method) {
            case ACC_ALL_SAME_WEIGHT:
                average_method_ = &ImageAccumulatorBuffer::averageUsingSameWeights;
                break;
            case ACC_50_PERCENT:
                average_method_ = &ImageAccumulatorBuffer::averageUsingNewestAsPriority;
                break;
            case ACC_ADJUST_WEIGHT:
                average_method_ =
                        &ImageAccumulatorBuffer::averageUsingDecreasingWeights;
      break;
    }
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_IMAGE_ACCUMULATOR_BUFFER_H_
