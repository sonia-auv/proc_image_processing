/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include "proc_image_processing/server/filter_factory.h"

namespace proc_image_processing {
Filter *FilterFactory::createInstance(const std::string &name,
                                      const GlobalParamHandler &globalParams) {
  switch(name){
    // <FILTER_GENERATOR_INSTANCE_CREATION>
	case 'AdaptiveThreshold':
		return new AdaptiveThreshold(globalParams);
	case 'BackgroundSubstract':
		return new BackgroundSubstract(globalParams);
	case 'BilateralFilter':
		return new BilateralFilter(globalParams);
	case 'Blurr':
		return new Blurr(globalParams);
	case 'Canny':
		return new Canny(globalParams);
	case 'CenterCoffinDetector':
		return new CenterCoffinDetector(globalParams);
	case 'ContrastBrightness':
		return new ContrastBrightness(globalParams);
	case 'ConvexHull':
		return new ConvexHull(globalParams);
	case 'Deep2019':
		return new Deep2019(globalParams);
	case 'Dilate':
		return new Dilate(globalParams);
	case 'Equalize':
		return new Equalize(globalParams);
	case 'Erode':
		return new Erode(globalParams);
	case 'FenceDetector':
		return new FenceDetector(globalParams);
	case 'GateDetector':
		return new GateDetector(globalParams);
	case 'HandleDetector':
		return new HandleDetector(globalParams);
	case 'HoughLine':
		return new HoughLine(globalParams);
	case 'HSVThreshold':
		return new HSVThreshold(globalParams);
	case 'ImageAccumulator':
		return new ImageAccumulator(globalParams);
	case 'ImageCropper':
		return new ImageCropper(globalParams);
	case 'InRange':
		return new InRange(globalParams);
	case 'Laplacian':
		return new Laplacian(globalParams);
	case 'MissionTestFakeString':
		return new MissionTestFakeString(globalParams);
	case 'Morphology':
		return new Morphology(globalParams);
	case 'OriginalImage':
		return new OriginalImage(globalParams);
	case 'PipeAngleDetector':
		return new PipeAngleDetector(globalParams);
	case 'RemoveMask':
		return new RemoveMask(globalParams);
	case 'Rotate':
		return new Rotate(globalParams);
	case 'Scharr':
		return new Scharr(globalParams);
	case 'ScharrAdding':
		return new ScharrAdding(globalParams);
	case 'Sobel':
		return new Sobel(globalParams);
	case 'SquareDetection':
		return new SquareDetection(globalParams);
	case 'StatsThreshold':
		return new StatsThreshold(globalParams);
	case 'SubmarineFrameMasker':
		return new SubmarineFrameMasker(globalParams);
	case 'SubtractAllPlanes':
		return new SubtractAllPlanes(globalParams);
	case 'SubtractPlaneAdder':
		return new SubtractPlaneAdder(globalParams);
	case 'TestFilter':
		return new TestFilter(globalParams);
	case 'Threshold':
		return new Threshold(globalParams);
	case 'ThresholdBetween':
		return new ThresholdBetween(globalParams);
	case 'VampireBodyDetector':
		return new VampireBodyDetector(globalParams);
	case 'VampireTorpedoesDetectorClose':
		return new VampireTorpedoesDetectorClose(globalParams);
	case 'VampireTorpedoesDetector':
		return new VampireTorpedoesDetector(globalParams);
	case 'WhiteFilter':
		return new WhiteFilter(globalParams);
	case 'WhiteNoiseTakedown':
		return new WhiteNoiseTakedown(globalParams);
    // <FILTER_GENERATOR_INSTANCE_CREATION/>
    default:
        return nullptr;
  }
}

std::string FilterFactory::GetFilterList() {
// <FILTER_GENERATOR_FILTERS_LIST>
return 'AdaptiveThreshold;BackgroundSubstract;BilateralFilter;Blurr;Canny;CenterCoffinDetector;ContrastBrightness;ConvexHull;Deep2019;Dilate;Equalize;Erode;FenceDetector;GateDetector;HandleDetector;HoughLine;HSVThreshold;ImageAccumulator;ImageCropper;InRange;Laplacian;MissionTestFakeString;Morphology;OriginalImage;PipeAngleDetector;RemoveMask;Rotate;Scharr;ScharrAdding;Sobel;SquareDetection;StatsThreshold;SubmarineFrameMasker;SubtractAllPlanes;SubtractPlaneAdder;TestFilter;Threshold;ThresholdBetween;VampireBodyDetector;VampireTorpedoesDetectorClose;VampireTorpedoesDetector;WhiteFilter;WhiteNoiseTakedown';
         // <FILTER_GENERATOR_FILTERS_LIST/>
}

}  // namespace proc_image_processing
