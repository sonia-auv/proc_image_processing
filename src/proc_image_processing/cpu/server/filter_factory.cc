#include "filter_factory.h"

namespace proc_image_processing {
    std::unique_ptr<Filter>
    FilterFactory::createInstance(const std::string_view &name, const GlobalParameterHandler &gph) {
        // <FACTORY_GENERATOR_INSTANCE_CREATION>
        if (name == "AccumulatorFilter") {
            return std::make_unique<AccumulatorFilter>(gph);
        } else if (name == "AdaptiveThresholdFilter") {
            return std::make_unique<AdaptiveThresholdFilter>(gph);
        } else if (name == "BackgroundSubtractFilter") {
            return std::make_unique<BackgroundSubtractFilter>(gph);
        } else if (name == "BilateralFilter") {
            return std::make_unique<BilateralFilter>(gph);
        } else if (name == "BlurFilter") {
            return std::make_unique<BlurFilter>(gph);
        } else if (name == "CannyFilter") {
            return std::make_unique<CannyFilter>(gph);
        } else if (name == "CenterCoffinDetector") {
            return std::make_unique<CenterCoffinDetector>(gph);
        } else if (name == "CLAHEFilter") {
            return std::make_unique<CLAHEFilter>(gph);
        } else if (name == "ContrastAndBrightnessFilter") {
            return std::make_unique<ContrastAndBrightnessFilter>(gph);
        } else if (name == "ConvexHullFilter") {
            return std::make_unique<ConvexHullFilter>(gph);
        } else if (name == "CropFilter") {
            return std::make_unique<CropFilter>(gph);
        } else if (name == "Deep2019Filter") {
            return std::make_unique<Deep2019Filter>(gph);
        } else if (name == "DilateFilter") {
            return std::make_unique<DilateFilter>(gph);
        } else if (name == "EqualizeHistogramFilter") {
            return std::make_unique<EqualizeHistogramFilter>(gph);
        } else if (name == "ErodeFilter") {
            return std::make_unique<ErodeFilter>(gph);
        } else if (name == "FenceDetector") {
            return std::make_unique<FenceDetector>(gph);
        } else if (name == "GateDetector") {
            return std::make_unique<GateDetector>(gph);
        } else if (name == "HandleDetector") {
            return std::make_unique<HandleDetector>(gph);
        } else if (name == "HideSubmarineFrameFilter") {
            return std::make_unique<HideSubmarineFrameFilter>(gph);
        } else if (name == "HoughLineFilter") {
            return std::make_unique<HoughLineFilter>(gph);
        } else if (name == "HSVThresholdFilter") {
            return std::make_unique<HSVThresholdFilter>(gph);
        } else if (name == "InRangeFilter") {
            return std::make_unique<InRangeFilter>(gph);
        } else if (name == "IntervalThresholdFilter") {
            return std::make_unique<IntervalThresholdFilter>(gph);
        } else if (name == "LaplacianFilter") {
            return std::make_unique<LaplacianFilter>(gph);
        } else if (name == "MissionTestFakeStringFilter") {
            return std::make_unique<MissionTestFakeStringFilter>(gph);
        } else if (name == "MorphologyFilter") {
            return std::make_unique<MorphologyFilter>(gph);
        } else if (name == "ObstacleDetector") {
            return std::make_unique<ObstacleDetector>(gph);
        } else if (name == "OriginalImageFilter") {
            return std::make_unique<OriginalImageFilter>(gph);
        } else if (name == "PipeAngleDetector") {
            return std::make_unique<PipeAngleDetector>(gph);
        } else if (name == "RemoveMaskFilter") {
            return std::make_unique<RemoveMaskFilter>(gph);
        } else if (name == "RotateFilter") {
            return std::make_unique<RotateFilter>(gph);
        } else if (name == "ScharrAddingFilter") {
            return std::make_unique<ScharrAddingFilter>(gph);
        } else if (name == "ScharrFilter") {
            return std::make_unique<ScharrFilter>(gph);
        } else if (name == "SobelFilter") {
            return std::make_unique<SobelFilter>(gph);
        } else if (name == "SquareDetector") {
            return std::make_unique<SquareDetector>(gph);
        } else if (name == "StatisticalThresholdFilter") {
            return std::make_unique<StatisticalThresholdFilter>(gph);
        } else if (name == "SubtractAllPlanesFilter") {
            return std::make_unique<SubtractAllPlanesFilter>(gph);
        } else if (name == "SubtractPlaneAdderFilter") {
            return std::make_unique<SubtractPlaneAdderFilter>(gph);
        } else if (name == "TestFilter") {
            return std::make_unique<TestFilter>(gph);
        } else if (name == "ThresholdFilter") {
            return std::make_unique<ThresholdFilter>(gph);
        } else if (name == "VampireBodyDetector") {
            return std::make_unique<VampireBodyDetector>(gph);
        } else if (name == "VampireTorpedoesCloseDetector") {
            return std::make_unique<VampireTorpedoesCloseDetector>(gph);
        } else if (name == "VampireTorpedoesDetector") {
            return std::make_unique<VampireTorpedoesDetector>(gph);
        } else if (name == "WhiteFilter") {
            return std::make_unique<WhiteFilter>(gph);
        } else if (name == "WhiteNoiseRemovalFilter") {
            return std::make_unique<WhiteNoiseRemovalFilter>(gph);
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
               "CLAHEFilter;"
               "ContrastAndBrightnessFilter;"
               "ConvexHullFilter;"
               "CropFilter;"
               "Deep2019Filter;"
               "DilateFilter;"
               "EqualizeHistogramFilter;"
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
               "ObstacleDetector"
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
