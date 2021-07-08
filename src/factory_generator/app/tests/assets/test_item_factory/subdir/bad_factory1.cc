switch(name){
    case 'AdaptiveThreshold':
		return new AdaptiveThreshold(globalParams);
	case 'BackgroundSubstract':
		return new BackgroundSubstract(globalParams);
	case 'BilateralFilter':
		return new BilateralFilter(globalParams);
    // <FACTORY_GENERATOR_INSTANCE_CREATION/>
}

std::string FilterFactory::GetFilterList() {
    // <FACTORY_GENERATOR_ITEMS_LIST>
	return 'AdaptiveThreshold;BackgroundSubstract;BilateralFilter;Blurr;Canny;CenterCoffinDetector;ContrastBrightness;ConvexHull;Deep2019;Dilate;Equalize;Erode;FenceDetector;GateDetector;HandleDetector;HoughLine;HSVThreshold;ImageAccumulator;ImageCropper;InRange;Laplacian;MissionTestFakeString;Morphology;OriginalImage;PipeAngleDetector;RemoveMask;Rotate;Scharr;ScharrAdding;Sobel;SquareDetection;StatsThreshold;SubmarineFrameMasker;SubtractAllPlanes;SubtractPlaneAdder;TestFilter;Threshold;ThresholdBetween;VampireBodyDetector;VampireTorpedoesDetectorClose;VampireTorpedoesDetector;WhiteFilter;WhiteNoiseTakedown';
    // <FACTORY_GENERATOR_ITEMS_LIST/>
}