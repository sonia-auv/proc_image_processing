/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_H_

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "type_and_const.h"

namespace proc_image_processing {

    class ObjectFeatureData {
    public:
        using Ptr = std::shared_ptr<ObjectFeatureData>;

        enum class Feature {
            RATIO,
            CONVEXITY,
            PERCENT_FILLED,
            CIRCULARITY,
            PRESENCE_CONSISTENCY,
            HUE_MEAN,
            SAT_MEAN,
            INTENSITY_MEAN,
            RED_MEAN,
            GREEN_MEAN,
            BLUE_MEAN,
            GRAY_MEAN
        };

        typedef std::vector<ObjectFeatureData::Ptr> ObjectFeatureVector;

        ObjectFeatureData()
                : ratio_(-1.0f),
                  convexity_(-1.0f),
                  percent_filled_(-1.0f),
                  circularity_(-1.0f),
                  presence_consistency_(-1.0f),
                  hue_mean_(-1.0f),
                  saturation_mean_(-1.0f),
                  intensity_mean_(-1.0f),
                  red_mean_(-1.0f),
                  green_mean_(-1.0f),
                  blue_mean_(-1.0f),
                  gray_mean_(-1.0f) {
        };

        virtual ~ObjectFeatureData() = default;

        inline float getRatio() const { return ratio_; }

        inline float getConvexity() const { return convexity_; }

        inline float getPercentFilled() const { return percent_filled_; }

        inline float getCircularity() const { return circularity_; }

        inline float getPresenceConsistency() const { return presence_consistency_; }

        inline float getHueMean() const { return hue_mean_; }

        inline float getSaturationMean() const { return saturation_mean_; }

        inline float getIntensityMean() const { return intensity_mean_; }

        inline float getRedMean() const { return red_mean_; }

        inline float getGreenMean() const { return green_mean_; }

        inline float getBlueMean() const { return blue_mean_; }

        inline float getGrayMean() const { return gray_mean_; }

        inline void setRatio(float value) { ratio_ = value; }

        inline void setConvexity(float value) { convexity_ = value; }

        inline void setPercentFilled(float value) { percent_filled_ = value; }

        inline void setCircularity(float value) { circularity_ = value; }

        inline void setPresenceConsistency(float value) { presence_consistency_ = value; }

        inline void setHueMean(float value) { hue_mean_ = value; }

        inline void setSatMean(float value) { saturation_mean_ = value; }

        inline void setIntensityMean(float value) { intensity_mean_ = value; }

        inline void setRedMean(float value) { red_mean_ = value; }

        inline void setGreenMean(float value) { green_mean_ = value; }

        inline void setBlueMean(float value) { blue_mean_ = value; }

        inline void setGrayMean(float value) { gray_mean_ = value; }


    private:
        float ratio_,
                convexity_,
                percent_filled_,
                circularity_,
                presence_consistency_,
                hue_mean_,
                saturation_mean_,
                intensity_mean_,
                red_mean_,
                green_mean_,
                blue_mean_,
                gray_mean_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_H_
