/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include "proc_image_processing/server/filter_factory.h"

namespace proc_image_processing {
Filter *FilterFactory::createInstance(const std::string &name,
                                      const GlobalParamHandler &globalParams) {
      // <FACTORY_GENERATOR_INSTANCE_CREATION>
	if(name == "AdaptiveThreshold"){
		return std::make_unique<AdaptiveThreshold>(globalParams);
	}
	else if(name == "BackgroundSubstract"){
		return std::make_unique<BackgroundSubstract>(globalParams);
	}
	else if(name == "BilateralFilter"){
		return std::make_unique<BilateralFilter>(globalParams);
	}
	else if(name == "Blurr"){
		return std::make_unique<Blurr>(globalParams);
	}
	else if(name == "Canny"){
		return std::make_unique<Canny>(globalParams);
	}
	else if(name == "CenterCoffinDetector"){
		return std::make_unique<CenterCoffinDetector>(globalParams);
	}
	else if(name == "ContrastBrightness"){
		return std::make_unique<ContrastBrightness>(globalParams);
	}
	else if(name == "ConvexHull"){
		return std::make_unique<ConvexHull>(globalParams);
	}
	else if(name == "Deep2019"){
		return std::make_unique<Deep2019>(globalParams);
	}
	else if(name == "Dilate"){
		return std::make_unique<Dilate>(globalParams);
	}
	else if(name == "Equalize"){
		return std::make_unique<Equalize>(globalParams);
	}
	else if(name == "Erode"){
		return std::make_unique<Erode>(globalParams);
	}
	else if(name == "FenceDetector"){
		return std::make_unique<FenceDetector>(globalParams);
	}
	else if(name == "GateDetector"){
		return std::make_unique<GateDetector>(globalParams);
	}
	else if(name == "HandleDetector"){
		return std::make_unique<HandleDetector>(globalParams);
	}
	else if(name == "HoughLine"){
		return std::make_unique<HoughLine>(globalParams);
	}
	else if(name == "HSVThreshold"){
		return std::make_unique<HSVThreshold>(globalParams);
	}
	else if(name == "ImageAccumulator"){
		return std::make_unique<ImageAccumulator>(globalParams);
	}
	else if(name == "ImageCropper"){
		return std::make_unique<ImageCropper>(globalParams);
	}
	else if(name == "InRange"){
		return std::make_unique<InRange>(globalParams);
	}
	else if(name == "Laplacian"){
		return std::make_unique<Laplacian>(globalParams);
	}
	else if(name == "MissionTestFakeString"){
		return std::make_unique<MissionTestFakeString>(globalParams);
	}
	else if(name == "Morphology"){
		return std::make_unique<Morphology>(globalParams);
	}
	else if(name == "OriginalImage"){
		return std::make_unique<OriginalImage>(globalParams);
	}
	else if(name == "PipeAngleDetector"){
		return std::make_unique<PipeAngleDetector>(globalParams);
	}
	else if(name == "RemoveMask"){
		return std::make_unique<RemoveMask>(globalParams);
	}
	else if(name == "Rotate"){
		return std::make_unique<Rotate>(globalParams);
	}
	else if(name == "Scharr"){
		return std::make_unique<Scharr>(globalParams);
	}
	else if(name == "ScharrAdding"){
		return std::make_unique<ScharrAdding>(globalParams);
	}
	else if(name == "Sobel"){
		return std::make_unique<Sobel>(globalParams);
	}
	else if(name == "SquareDetection"){
		return std::make_unique<SquareDetection>(globalParams);
	}
	else if(name == "StatsThreshold"){
		return std::make_unique<StatsThreshold>(globalParams);
	}
	else if(name == "SubmarineFrameMasker"){
		return std::make_unique<SubmarineFrameMasker>(globalParams);
	}
	else if(name == "SubtractAllPlanes"){
		return std::make_unique<SubtractAllPlanes>(globalParams);
	}
	else if(name == "TestFilter"){
		return std::make_unique<TestFilter>(globalParams);
	}
	else if(name == "Threshold"){
		return std::make_unique<Threshold>(globalParams);
	}
	else if(name == "ThresholdBetween"){
		return std::make_unique<ThresholdBetween>(globalParams);
	}
	else if(name == "VampireBodyDetector"){
		return std::make_unique<VampireBodyDetector>(globalParams);
	}
	else if(name == "VampireTorpedoesDetectorClose"){
		return std::make_unique<VampireTorpedoesDetectorClose>(globalParams);
	}
	else if(name == "VampireTorpedoesDetector"){
		return std::make_unique<VampireTorpedoesDetector>(globalParams);
	}
	else if(name == "WhiteFilter"){
		return std::make_unique<WhiteFilter>(globalParams);
	}
	else if(name == "WhiteNoiseTakedown"){
		return std::make_unique<WhiteNoiseTakedown>(globalParams);
	}
          // <FACTORY_GENERATOR_INSTANCE_CREATION/>
    else{
        return nullptr;
    }
}

std::string FilterFactory::GetFilterList() {
// <FACTORY_GENERATOR_ITEMS_LIST>
	return "AdaptiveThreshold;"
		"BackgroundSubstract;"
		"BilateralFilter;"
		"Blurr;"
		"Canny;"
		"CenterCoffinDetector;"
		"ContrastBrightness;"
		"ConvexHull;"
		"Deep2019;"
		"Dilate;"
		"Equalize;"
		"Erode;"
		"FenceDetector;"
		"GateDetector;"
		"HandleDetector;"
		"HoughLine;"
		"HSVThreshold;"
		"ImageAccumulator;"
		"ImageCropper;"
		"InRange;"
		"Laplacian;"
		"MissionTestFakeString;"
		"Morphology;"
		"OriginalImage;"
		"PipeAngleDetector;"
		"RemoveMask;"
		"Rotate;"
		"Scharr;"
		"ScharrAdding;"
		"Sobel;"
		"SquareDetection;"
		"StatsThreshold;"
		"SubmarineFrameMasker;"
		"SubtractAllPlanes;"
		"TestFilter;"
		"Threshold;"
		"ThresholdBetween;"
		"VampireBodyDetector;"
		"VampireTorpedoesDetectorClose;"
		"VampireTorpedoesDetector;"
		"WhiteFilter;"
		"WhiteNoiseTakedown";
         // <FACTORY_GENERATOR_ITEMS_LIST/>
}

}  // namespace proc_image_processing
