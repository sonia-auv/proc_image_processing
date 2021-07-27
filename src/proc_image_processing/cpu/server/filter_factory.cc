#include "filter_factory.h"

namespace proc_image_processing {
    std::unique_ptr<Filter>
    FilterFactory::createInstance(const std::string &name, const GlobalParamHandler &globalParams) {
        // <FACTORY_GENERATOR_INSTANCE_CREATION>
        if (name == "AccumulatorFilter") {
            return std::move(std::make_unique<AccumulatorFilter>(globalParams));
        } else if (name == "AdaptiveThresholdFilter") {
            return std::move(std::make_unique<AdaptiveThresholdFilter>(globalParams));
        } else if (name == "BackgroundSubtractFilter") {
            return std::move(std::make_unique<BackgroundSubtractFilter>(globalParams));
        } else if (name == "BilateralFilter") {
            return std::move(std::make_unique<BilateralFilter>(globalParams));
        } else if (name == "BlurFilter") {
            return std::move(std::make_unique<BlurFilter>(globalParams));
        } else if (name == "CannyFilter") {
            return std::move(std::make_unique<CannyFilter>(globalParams));
        } else if (name == "CenterCoffinDetector") {
            return std::move(std::make_unique<CenterCoffinDetector>(globalParams));
        } else if (name == "ContrastAndBrightnessFilter") {
            return std::move(std::make_unique<ContrastAndBrightnessFilter>(globalParams));
        } else if (name == "ConvexHullFilter") {
            return std::move(std::make_unique<ConvexHullFilter>(globalParams));
        } else if (name == "CropFilter") {
            return std::move(std::make_unique<CropFilter>(globalParams));
        } else if (name == "Deep2019Filter") {
            return std::move(std::make_unique<Deep2019Filter>(globalParams));
        } else if (name == "DilateFilter") {
            return std::move(std::make_unique<DilateFilter>(globalParams));
        } else if (name == "EqualizeFilter") {
            return std::move(std::make_unique<EqualizeFilter>(globalParams));
        } else if (name == "ErodeFilter") {
            return std::move(std::make_unique<ErodeFilter>(globalParams));
        } else if (name == "FenceDetector") {
            return std::move(std::make_unique<FenceDetector>(globalParams));
        } else if (name == "GateDetector") {
            return std::move(std::make_unique<GateDetector>(globalParams));
        } else if (name == "HandleDetector") {
            return std::move(std::make_unique<HandleDetector>(globalParams));
        } else if (name == "HideSubmarineFrameFilter") {
            return std::move(std::make_unique<HideSubmarineFrameFilter>(globalParams));
        } else if (name == "HoughLineFilter") {
            return std::move(std::make_unique<HoughLineFilter>(globalParams));
        } else if (name == "HSVThresholdFilter") {
            return std::move(std::make_unique<HSVThresholdFilter>(globalParams));
        } else if (name == "InRangeFilter") {
            return std::move(std::make_unique<InRangeFilter>(globalParams));
        } else if (name == "IntervalThresholdFilter") {
            return std::move(std::make_unique<IntervalThresholdFilter>(globalParams));
        } else if (name == "LaplacianFilter") {
            return std::move(std::make_unique<LaplacianFilter>(globalParams));
        } else if (name == "MissionTestFakeStringFilter") {
            return std::move(std::make_unique<MissionTestFakeStringFilter>(globalParams));
        } else if (name == "MorphologyFilter") {
            return std::move(std::make_unique<MorphologyFilter>(globalParams));
        } else if (name == "OriginalImageFilter") {
            return std::move(std::make_unique<OriginalImageFilter>(globalParams));
        } else if (name == "PipeAngleDetector") {
            return std::move(std::make_unique<PipeAngleDetector>(globalParams));
        } else if (name == "RemoveMaskFilter") {
            return std::move(std::make_unique<RemoveMaskFilter>(globalParams));
        } else if (name == "RotateFilter") {
            return std::move(std::make_unique<RotateFilter>(globalParams));
        } else if (name == "ScharrAddingFilter") {
            return std::move(std::make_unique<ScharrAddingFilter>(globalParams));
        } else if (name == "ScharrFilter") {
            return std::move(std::make_unique<ScharrFilter>(globalParams));
        } else if (name == "SobelFilter") {
            return std::move(std::make_unique<SobelFilter>(globalParams));
        } else if (name == "SquareDetector") {
            return std::move(std::make_unique<SquareDetector>(globalParams));
        } else if (name == "StatisticalThresholdFilter") {
            return std::move(std::make_unique<StatisticalThresholdFilter>(globalParams));
        } else if (name == "SubtractAllPlanesFilter") {
            return std::move(std::make_unique<SubtractAllPlanesFilter>(globalParams));
        } else if (name == "SubtractPlaneAdderFilter") {
            return std::move(std::make_unique<SubtractPlaneAdderFilter>(globalParams));
        } else if (name == "TestFilter") {
            return std::move(std::make_unique<TestFilter>(globalParams));
        } else if (name == "ThresholdFilter") {
            return std::move(std::make_unique<ThresholdFilter>(globalParams));
        } else if (name == "VampireBodyDetector") {
            return std::move(std::make_unique<VampireBodyDetector>(globalParams));
        } else if (name == "VampireTorpedoesCloseDetector") {
            return std::move(std::make_unique<VampireTorpedoesCloseDetector>(globalParams));
        } else if (name == "VampireTorpedoesDetector") {
            return std::move(std::make_unique<VampireTorpedoesDetector>(globalParams));
        } else if (name == "WhiteFilter") {
            return std::move(std::make_unique<WhiteFilter>(globalParams));
        } else if (name == "WhiteNoiseRemovalFilter") {
            return std::move(std::make_unique<WhiteNoiseRemovalFilter>(globalParams));
        }
            // <FACTORY_GENERATOR_INSTANCE_CREATION/>
        else {
            return nullptr;
        }
    }

    std::string FilterFactory::getFilters() {
        // <FACTORY_GENERATOR_ITEMS_LIST>
        return "AccumulatorFilter;"
               "AdaptiveThresholdFilter;"
               "BackgroundSubtractFilter;"
               "BilateralFilter;"
               "BlurFilter;"
               "CannyFilter;"
               "CenterCoffinDetector;"
               "ContrastAndBrightnessFilter;"
               "ConvexHullFilter;"
               "CropFilter;"
               "Deep2019Filter;"
               "DilateFilter;"
               "EqualizeFilter;"
               "ErodeFilter;"
               "FenceDetector;"
               "GateDetector;"
               "HandleDetector;"
               "HideSubmarineFrameFilter;"
               "HoughLineFilter;"
               "HSVThresholdFilter;"
               "InRangeFilter;"
               "IntervalThresholdFilter;"
               "LaplacianFilter;"
               "MissionTestFakeStringFilter;"
               "MorphologyFilter;"
               "OriginalImageFilter;"
               "PipeAngleDetector;"
               "RemoveMaskFilter;"
               "RotateFilter;"
               "ScharrAddingFilter;"
               "ScharrFilter;"
               "SobelFilter;"
               "SquareDetector;"
               "StatisticalThresholdFilter;"
               "SubtractAllPlanesFilter;"
               "SubtractPlaneAdderFilter;"
               "TestFilter;"
               "ThresholdFilter;"
               "VampireBodyDetector;"
               "VampireTorpedoesCloseDetector;"
               "VampireTorpedoesDetector;"
               "WhiteFilter;"
               "WhiteNoiseRemovalFilter";
        // <FACTORY_GENERATOR_ITEMS_LIST/>
    }

}  // namespace proc_image_processing
