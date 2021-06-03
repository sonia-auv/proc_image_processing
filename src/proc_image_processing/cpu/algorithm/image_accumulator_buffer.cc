/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include <proc_image_processing/algorithm/image_accumulator_buffer.h>

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ImageAccumulatorBuffer::ImageAccumulatorBuffer(int bufferLength,
                                               cv::Size imgSize, int type,
                                               METHOD method)
    : buffer_size_(bufferLength),
      individual_weight_(0.0f),
      buffer_current_index_(0),
      image_vec_(0),
      image_type_(type),
      image_size_(imgSize),
      average_method_(nullptr) {
  // Start with a buffer filled with blank matrices.
  FillWithBlank();
  individual_weight_ = 1.0 / static_cast<float>(bufferLength);
  SetAverageMethod(method);
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::GetImage(cv::Mat &image) {
  // We do not want to access blank...&
  if (buffer_size_ == 0) return;
  // Use the function in memory to average the accumulator and
  // reset the values
  (this->*average_method_)(image);
  image.convertTo(image, image_type_);
}

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::AddImage(const cv::Mat &image) {
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

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::ResetBuffer() { FillWithBlank(); }

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::ResetBuffer(int bufferLength, cv::Size imgSize,
                                         int type) {
  buffer_size_ = bufferLength;
  image_size_ = imgSize;
  image_type_ = type;
  buffer_current_index_ = 0;
  individual_weight_ = 1.0 / static_cast<float>(bufferLength);

  FillWithBlank();
}

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::AverageAllSameWeight(cv::Mat &resultImage) {
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

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::AverageIncrease50Percent(cv::Mat &resultImage) {
  resultImage = image_vec_[GetIndexFromOldest(0)].clone();
  for (size_t i = 1; i < buffer_size_; i++) {
    cv::addWeighted(resultImage, 0.5, image_vec_[GetIndexFromOldest(i)], 0.5, 0,
                    resultImage);
  }
}

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::AverageAccumulateWithResultingWeight(
    cv::Mat &resultImage) {
  // Fill resultImage
  // Keep in float to stay at full scale.
  resultImage = cv::Mat::zeros(image_size_, CV_32FC(CV_MAT_CN(image_type_)));
  cv::addWeighted(resultImage, 0.5, image_vec_[GetIndexFromMostRecent(0)], 0.5,
                  0, resultImage);

  float resultingWeight = 0.50f;
  // Start at one, we already did the 0 index (newest)
  for (size_t i = 1; i < buffer_size_; i++) {
    // Addjust the weight, we are one element older now then before,
    // so the weight of this element is half the precendent.
    resultingWeight = resultingWeight / 2;

    cv::add(resultImage,
            image_vec_[GetIndexFromMostRecent(i)] * resultingWeight,
            resultImage);
  }
}

}  // namespace proc_image_processing
