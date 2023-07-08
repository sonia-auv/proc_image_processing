#include "filter_factory.h"

namespace proc_image_processing {
    std::unique_ptr<Filter>
    FilterFactory::createInstance(const std::string_view &name, const GlobalParameterHandler &gph) {
        // <FACTORY_GENERATOR_INSTANCE_CREATION>
	if(name == "AccumulatorFilter"){
		return std::make_unique<AccumulatorFilter>(gph);
	}
	else if(name == "AdaptiveThresholdFilter"){
		return std::make_unique<AdaptiveThresholdFilter>(gph);
	}
	else if(name == "BackgroundSubtractFilter"){
		return std::make_unique<BackgroundSubtractFilter>(gph);
	}
	else if(name == "BGRFilter"){
		return std::make_unique<BGRFilter>(gph);
	}
	else if(name == "BilateralFilter"){
		return std::make_unique<BilateralFilter>(gph);
	}
	else if(name == "BlurFilter"){
		return std::make_unique<BlurFilter>(gph);
	}
	else if(name == "BoundingBoxFilter"){
		return std::make_unique<BoundingBoxFilter>(gph);
	}
	else if(name == "CannyFilter"){
		return std::make_unique<CannyFilter>(gph);
	}
	else if(name == "CenterDetector"){
		return std::make_unique<CenterDetector>(gph);
	}
	else if(name == "CLAHEFilter"){
		return std::make_unique<CLAHEFilter>(gph);
	}
	else if(name == "ContourDetector"){
		return std::make_unique<ContourDetector>(gph);
	}
	else if(name == "ContrastAndBrightnessFilter"){
		return std::make_unique<ContrastAndBrightnessFilter>(gph);
	}
	else if(name == "ConvexHullFilter"){
		return std::make_unique<ConvexHullFilter>(gph);
	}
	else if(name == "CropFilter"){
		return std::make_unique<CropFilter>(gph);
	}
	else if(name == "DeepFilter"){
		return std::make_unique<DeepFilter>(gph);
	}
	else if(name == "DilateFilter"){
		return std::make_unique<DilateFilter>(gph);
	}
	else if(name == "EllipseDetector"){
		return std::make_unique<EllipseDetector>(gph);
	}
	else if(name == "EqualizeHistogramFilter"){
		return std::make_unique<EqualizeHistogramFilter>(gph);
	}
	else if(name == "ErodeFilter"){
		return std::make_unique<ErodeFilter>(gph);
	}
	else if(name == "FenceDetector"){
		return std::make_unique<FenceDetector>(gph);
	}
	else if(name == "GateDetector"){
		return std::make_unique<GateDetector>(gph);
	}
	else if(name == "GateBlobDetector"){
		return std::make_unique<GateBlobDetector>(gph);
	}
	else if(name == "GateSymbolDetector"){
		return std::make_unique<GateSymbolDetector>(gph);
	}
	else if(name == "HandleDetector"){
		return std::make_unique<HandleDetector>(gph);
	}
	else if(name == "HideSubmarineFrameFilter"){
		return std::make_unique<HideSubmarineFrameFilter>(gph);
	}
	else if(name == "HoughLineFilter"){
		return std::make_unique<HoughLineFilter>(gph);
	}
	else if(name == "HSVFilter"){
		return std::make_unique<HSVFilter>(gph);
	}
	else if(name == "HSVThresholdFilter"){
		return std::make_unique<HSVThresholdFilter>(gph);
	}
	else if(name == "InRangeFilter"){
		return std::make_unique<InRangeFilter>(gph);
	}
	else if(name == "IntervalThresholdFilter"){
		return std::make_unique<IntervalThresholdFilter>(gph);
	}
	else if(name == "LABFilter"){
		return std::make_unique<LABFilter>(gph);
	}
	else if(name == "LaplacianFilter"){
		return std::make_unique<LaplacianFilter>(gph);
	}
	else if(name == "MissionTestFakeStringFilter"){
		return std::make_unique<MissionTestFakeStringFilter>(gph);
	}
	else if(name == "MorphologyFilter"){
		return std::make_unique<MorphologyFilter>(gph);
	}
	else if(name == "ObstacleDetector"){
		return std::make_unique<ObstacleDetector>(gph);
	}
	else if(name == "OrbSiftMatch"){
		return std::make_unique<OrbSiftMatch>(gph);
	}
	else if(name == "OriginalImageFilter"){
		return std::make_unique<OriginalImageFilter>(gph);
	}
	else if(name == "PipeAngleDetector"){
		return std::make_unique<PipeAngleDetector>(gph);
	}
	else if(name == "PipeStraightDetector"){
		return std::make_unique<PipeStraightDetector>(gph);
	}
	else if(name == "RemoveMaskFilter"){
		return std::make_unique<RemoveMaskFilter>(gph);
	}
	else if(name == "RotateFilter"){
		return std::make_unique<RotateFilter>(gph);
	}
	else if(name == "ScharrAddingFilter"){
		return std::make_unique<ScharrAddingFilter>(gph);
	}
	else if(name == "ScharrFilter"){
		return std::make_unique<ScharrFilter>(gph);
	}
	else if(name == "ShapeDetector"){
		return std::make_unique<ShapeDetector>(gph);
	}
	else if(name == "SiftMatch"){
		return std::make_unique<SiftMatch>(gph);
	}
	else if(name == "SiftCalculator"){
		return std::make_unique<SiftCalculator>(gph);
	}
		else if(name == "SimpleBlob"){
		return std::make_unique<SimpleBlob>(gph);
	}
	else if(name == "SingleColorFilter"){
		return std::make_unique<SingleColorFilter>(gph);
	}
	else if(name == "SobelFilter"){
		return std::make_unique<SobelFilter>(gph);
	}
	else if(name == "SquareDetector"){
		return std::make_unique<SquareDetector>(gph);
	}
	else if(name == "StatisticalThresholdFilter"){
		return std::make_unique<StatisticalThresholdFilter>(gph);
	}
	else if(name == "SubtractAllPlanesFilter"){
		return std::make_unique<SubtractAllPlanesFilter>(gph);
	}
	else if(name == "SubtractPlaneAdderFilter"){
		return std::make_unique<SubtractPlaneAdderFilter>(gph);
	}
	else if(name == "TestFilter"){
		return std::make_unique<TestFilter>(gph);
	}
	else if(name == "ThresholdFilter"){
		return std::make_unique<ThresholdFilter>(gph);
	}
	else if(name == "VampireBodyDetector"){
		return std::make_unique<VampireBodyDetector>(gph);
	}
	else if(name == "VampireTorpedoesCloseDetector"){
		return std::make_unique<VampireTorpedoesCloseDetector>(gph);
	}
	else if(name == "VampireTorpedoesDetector"){
		return std::make_unique<VampireTorpedoesDetector>(gph);
	}
	else if(name == "WhiteFilter"){
		return std::make_unique<WhiteFilter>(gph);
	}
	else if(name == "WhiteNoiseRemovalFilter"){
		return std::make_unique<WhiteNoiseRemovalFilter>(gph);
	}
	else if(name == "YCrCbFilter"){
		return std::make_unique<YCrCbFilter>(gph);
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
		"BGRFilter;"
		"BilateralFilter;"
		"BlurFilter;"
		"BoundingBoxFilter;"
		"CannyFilter;"
		"CenterDetector;"
		"CLAHEFilter;"
		"ContourDetector;"
		"ContrastAndBrightnessFilter;"
		"ConvexHullFilter;"
		"CropFilter;"
		"DeepFilter;"
		"DilateFilter;"
		"EllipseDetector;"
		"EqualizeHistogramFilter;"
		"ErodeFilter;"
		"FenceDetector;"
		"GateDetector;"
		"GateBlobDetector;"
		"GateSymbolDetector;"
		"HandleDetector;"
		"HideSubmarineFrameFilter;"
		"HoughLineFilter;"
		"HSVFilter;"
		"HSVThresholdFilter;"
		"InRangeFilter;"
		"IntervalThresholdFilter;"
		"LABFilter;"
		"LaplacianFilter;"
		"MissionTestFakeStringFilter;"
		"MorphologyFilter;"
		"ObstacleDetector;"
		"OrbSiftMatch;"
		"OriginalImageFilter;"
		"PipeAngleDetector;"
		"PipeStraightDetector;"
		"RemoveMaskFilter;"
		"RotateFilter;"
		"ScharrAddingFilter;"
		"ScharrFilter;"
		"ShapeDetector;"
		"SiftMatch;"
		"SiftCalculator;"
		"SimpleBlob;"
		"SingleColorFilter;"
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
		"WhiteNoiseRemovalFilter;"
		"YCrCbFilter";
        // <FACTORY_GENERATOR_ITEMS_LIST/>
    }

}  // namespace proc_image_processing
