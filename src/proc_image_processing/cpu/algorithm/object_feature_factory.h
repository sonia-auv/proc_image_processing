#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_FACTORY_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_FACTORY_H_

#include <map>
#include <memory>
#include <vector>
#include "object_feature.h"
#include "object_frame_memory.h"
#include "object_full_data.h"

namespace proc_image_processing {

    class ObjectFeatureFactory {
    public:
        using Ptr = std::shared_ptr<ObjectFeatureFactory>;

        /**
         * Defines the type "Pointer to method to calculate a feature"
         * typedef float
         * (ObjectFeatureFactory::*FeatureFunction)(std::shared_ptr<ObjectFullData>);
         */
        typedef std::function<void(ObjectFullData::Ptr)> FeatureFunction;

        explicit ObjectFeatureFactory(unsigned int memorySize);

        ~ObjectFeatureFactory() = default;

        void computeAllFeature(const ObjectFullData::FullObjectPtrVec &objects);

        void computeAllFeature(const ObjectFullData::Ptr &object);

        void computeSelectedFeature(const ObjectFullData::FullObjectPtrVec &objects,
                                    const std::vector<ObjectFeatureData::Feature> &feature);

        void computeSelectedFeature(const ObjectFullData::Ptr &object,
                                    const std::vector<ObjectFeatureData::Feature> &feature);

        // feature functions
        static void ratioFeature(const ObjectFullData::Ptr &object);

        static void convexityFeature(const ObjectFullData::Ptr &object);

        static void percentFilledFeature(const ObjectFullData::Ptr &object);

        static void circularityFeature(const ObjectFullData::Ptr &object);

        void presenceConsistencyFeature(const ObjectFullData::Ptr &object);

        static void hueMeanFeature(const ObjectFullData::Ptr &object);

        static void saturationMeanFeature(const ObjectFullData::Ptr &object);

        static void intensityMeanFeature(const ObjectFullData::Ptr &object);

        static void redMeanFeature(const ObjectFullData::Ptr &object);

        static void greenMeanFeature(const ObjectFullData::Ptr &object);

        static void blueMeanFeature(const ObjectFullData::Ptr &object);

        static void grayMeanFeature(const ObjectFullData::Ptr &object);

    private:
        static float calculatePlaneMean(const ObjectFullData::Ptr &object, int plane);

        // the vector of function enables iterating through the function that needs
        // to be call in order to compute the good feature.
        std::map<ObjectFeatureData::Feature, FeatureFunction> feature_fct_map_;

        ObjectFrameMemory frame_memory_;
    };

    inline void ObjectFeatureFactory::computeAllFeature(
            const ObjectFullData::FullObjectPtrVec &objects) {
        for (auto &object : objects) {
            computeAllFeature(object);
        }
    }

    inline void ObjectFeatureFactory::computeAllFeature(const ObjectFullData::Ptr &object) {
        for (auto &fct : feature_fct_map_) {
            fct.second(object);  //(*(fct.second));
            //*tmp(object);
        }
    }

    inline void ObjectFeatureFactory::computeSelectedFeature(
            const ObjectFullData::FullObjectPtrVec &objects,
            const std::vector<ObjectFeatureData::Feature> &feature) {
        for (auto &object : objects) {
            computeSelectedFeature(object, feature);
        }
    }

    inline void ObjectFeatureFactory::computeSelectedFeature(
            const ObjectFullData::Ptr &object,
            const std::vector<ObjectFeatureData::Feature> &feature) {
        for (const auto &feat : feature) {
            (feature_fct_map_[feat])(object);
        }
    }

    inline void ObjectFeatureFactory::ratioFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getRatio() == -1.0f)) {
            RotRect rect = object->getRotRect();
            object->setRatio(rect.size.width / rect.size.height);
        }
    }

    inline void ObjectFeatureFactory::convexityFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getConvexity() == -1.0f)) {
            // safety, should not happen
            float convexHull = object->getConvexHullArea();
            float area = object->getArea();
            if (convexHull > 0 && area > 0)
                object->setConvexity(1.0f - (area / convexHull));
        }
    }

    inline void ObjectFeatureFactory::circularityFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getCircularity() == -1.0f)) {
            // Here we use pow on radius instead of sqrt on area because
            // pow is less hard computation
            float radiusCircum = pow(object->getCircumference() / (2 * M_PI), 2);
            float radiusArea = object->getArea() / (M_PI);
            if (radiusCircum != 0 && radiusArea != 0) {
                object->setCircularity(radiusCircum > radiusArea
                                       ? radiusArea / radiusCircum
                                       : radiusCircum / radiusArea);
            }
        }
    }

    inline void ObjectFeatureFactory::presenceConsistencyFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) &&
            (object->getPresenceConsistency() == -1.0f)) {
            float ratio = object->getRatio();
            if (ratio == -1.0f) {
                ratioFeature(object);
                ratio = object->getRatio();
            }
            ObjectFullData::FullObjectPtrVec vec =
                    frame_memory_.GetPastObjectsViaCenter(object->getCenterPoint(), ratio);
            object->setPresenceConsistency(float(vec.size()) /
                                           float(frame_memory_.GetMemorySize()));
        }
    }

    inline void ObjectFeatureFactory::hueMeanFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getHueMean() == -1.0f)) {
            object->setHueMean(calculatePlaneMean(object, ObjectBasicData::HUE_PLANE));
        }
    }

    inline void ObjectFeatureFactory::saturationMeanFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getSaturationMean() == -1.0f)) {
            object->setSatMean(
                    calculatePlaneMean(object, ObjectBasicData::SATURATION_PLANE));
        }
    }

    inline void ObjectFeatureFactory::intensityMeanFeature(const std::shared_ptr<ObjectFullData> &object) {
        if ((object != nullptr) && (object->getIntensityMean() == -1.0f)) {
            object->setIntensityMean(
                    calculatePlaneMean(object, ObjectBasicData::INTENSITY_PLANE));
        }
    }

    inline void ObjectFeatureFactory::redMeanFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getRedMean() == -1.0f)) {
            object->setRedMean(calculatePlaneMean(object, ObjectBasicData::RED_PLANE));
        }
    }

    inline void ObjectFeatureFactory::greenMeanFeature(const std::shared_ptr<ObjectFullData> &object) {
        if ((object != nullptr) && (object->getGreenMean() == -1.0f)) {
            object->setGreenMean(
                    calculatePlaneMean(object, ObjectBasicData::GREEN_PLANE));
        }
    }

    inline void ObjectFeatureFactory::blueMeanFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getBlueMean() == -1.0f)) {
            object->setBlueMean(
                    calculatePlaneMean(object, ObjectBasicData::BLUE_PLANE));
        }
    }

    inline void ObjectFeatureFactory::grayMeanFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getGrayMean() == -1.0f)) {
            object->setGrayMean(
                    calculatePlaneMean(object, ObjectBasicData::GRAY_PLANE));
        }
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_FEATURE_FACTORY_H_
