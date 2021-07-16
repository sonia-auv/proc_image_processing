/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_H_

#include <stdlib.h>
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
                  sat_mean_(-1.0f),
                  intensity_mean_(-1.0f),
                  red_mean_(-1.0f),
                  green_mean_(-1.0f),
                  blue_mean_(-1.0f),
                  gray_mean_(-1.0f) {
        };

        virtual ~ObjectFeatureData() {};

        inline float GetRatio() const { return ratio_; }

        inline float GetConvexity() const { return convexity_; }

        inline float GetPercentFilled() const { return percent_filled_; }

        inline float GetCircularity() const { return circularity_; }

        inline float GetPresenceConsistency() const { return presence_consistency_; }

        inline float GetHueMean() const { return hue_mean_; }

        inline float GetSatMean() const { return sat_mean_; }

        inline float GetIntensityMean() const { return intensity_mean_; }

        inline float GetRedMean() const { return red_mean_; }

        inline float GetGreenMean() const { return green_mean_; }

        inline float GetBlueMean() const { return blue_mean_; }

        inline float GetGrayMean() const { return gray_mean_; }

        inline void SetRatio(float value) { ratio_ = value; }

        inline void SetConvexity(float value) { convexity_ = value; }

        inline void SetPercentFilled(float value) { percent_filled_ = value; }

        inline void SetCircularity(float value) { circularity_ = value; }

        inline void SetPresenceConsistency(float value) {
            presence_consistency_ = value;
        }

        inline void SetHueMean(float value) { hue_mean_ = value; }

        inline void SetSatMean(float value) { sat_mean_ = value; }

        inline void SetIntensityMean(float value) { intensity_mean_ = value; }

        inline void SetRedMean(float value) { red_mean_ = value; }

        inline void SetGreenMean(float value) { green_mean_ = value; }

        inline void SetBlueMean(float value) { blue_mean_ = value; }

        inline void SetGrayMean(float value) { gray_mean_ = value; }


    private:
        float ratio_, convexity_, percent_filled_, circularity_,
                presence_consistency_, hue_mean_, sat_mean_, intensity_mean_, red_mean_,
                green_mean_, blue_mean_, gray_mean_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_H_
