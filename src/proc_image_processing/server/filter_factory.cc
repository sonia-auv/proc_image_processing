/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include "proc_image_processing/server/filter_factory.h"

namespace proc_image_processing {
Filter *FilterFactory::createInstance(const std::string &name,
                                      const GlobalParamHandler &globalParams) {
  switch (std::hash(name)) {
      // <FACTORY_GENERATOR_INSTANCE_CREATION>
      case std::hash("AdaptiveThreshold"):
          return new AdaptiveThreshold(globalParams);
      case std::hash("BackgroundSubstract"):
          return new BackgroundSubstract(globalParams);
      case std::hash("BilateralFilter"):
          return new BilateralFilter(globalParams);
      case std::hash("Blurr"):
          return new Blurr(globalParams);
      case std::hash("Canny"):
          return new Canny(globalParams);
      case std::hash("CenterCoffinDetector"):
          return new CenterCoffinDetector(globalParams);
      case std::hash("ContrastBrightness"):
          return new ContrastBrightness(globalParams);
      case std::hash("ConvexHull"):
          return new ConvexHull(globalParams);
      case std::hash("Deep2019"):
          return new Deep2019(globalParams);
      case std::hash("Dilate"):
          return new Dilate(globalParams);
      case std::hash("Equalize"):
          return new Equalize(globalParams);
      case std::hash("Erode"):
          return new Erode(globalParams);
      case std::hash("FenceDetector"):
          return new FenceDetector(globalParams);
      case std::hash("GateDetector"):
          return new GateDetector(globalParams);
      case std::hash("HandleDetector"):
          return new HandleDetector(globalParams);
      case std::hash("HoughLine"):
          return new HoughLine(globalParams);
      case std::hash("HSVThreshold"):
          return new HSVThreshold(globalParams);
      case std::hash("ImageAccumulator"):
          return new ImageAccumulator(globalParams);
      case std::hash("ImageCropper"):
          return new ImageCropper(globalParams);
      case std::hash("InRange"):
          return new InRange(globalParams);
      case std::hash("Laplacian"):
          return new Laplacian(globalParams);
      case std::hash("MissionTestFakeString"):
          return new MissionTestFakeString(globalParams);
      case std::hash("Morphology"):
          return new Morphology(globalParams);
      case std::hash("OriginalImage"):
          return new OriginalImage(globalParams);
      case std::hash("PipeAngleDetector"):
          return new PipeAngleDetector(globalParams);
      case std::hash("RemoveMask"):
          return new RemoveMask(globalParams);
      case std::hash("Rotate"):
          return new Rotate(globalParams);
      case std::hash("Scharr"):
          return new Scharr(globalParams);
      case std::hash("ScharrAdding"):
          return new ScharrAdding(globalParams);
      case std::hash("Sobel"):
          return new Sobel(globalParams);
      case std::hash("SquareDetection"):
          return new SquareDetection(globalParams);
      case std::hash("StatsThreshold"):
          return new StatsThreshold(globalParams);
      case std::hash("SubmarineFrameMasker"):
          return new SubmarineFrameMasker(globalParams);
      case std::hash("SubtractAllPlanes"):
          return new SubtractAllPlanes(globalParams);
      case std::hash("TestFilter"):
          return new TestFilter(globalParams);
      case std::hash("Threshold"):
          return new Threshold(globalParams);
      case std::hash("ThresholdBetween"):
          return new ThresholdBetween(globalParams);
      case std::hash("VampireBodyDetector"):
          return new VampireBodyDetector(globalParams);
      case std::hash("VampireTorpedoesDetectorClose"):
          return new VampireTorpedoesDetectorClose(globalParams);
      case std::hash("VampireTorpedoesDetector"):
          return new VampireTorpedoesDetector(globalParams);
      case std::hash("WhiteFilter"):
          return new WhiteFilter(globalParams);
      case std::hash("WhiteNoiseTakedown"):
          return new WhiteNoiseTakedown(globalParams);
          // <FACTORY_GENERATOR_INSTANCE_CREATION/>
      default:
          return nullptr;
  }
}

std::string FilterFactory::GetFilterList() {
// <FACTORY_GENERATOR_ITEMS_LIST>
	return "AdaptiveThreshold;BackgroundSubstract;BilateralFilter;Blurr;Canny;CenterCoffinDetector;ContrastBrightness;ConvexHull;Deep2019;Dilate;Equalize;Erode;FenceDetector;GateDetector;HandleDetector;HoughLine;HSVThreshold;ImageAccumulator;ImageCropper;InRange;Laplacian;MissionTestFakeString;Morphology;OriginalImage;PipeAngleDetector;RemoveMask;Rotate;Scharr;ScharrAdding;Sobel;SquareDetection;StatsThreshold;SubmarineFrameMasker;SubtractAllPlanes;SubtractPlaneAdder;TestFilter;Threshold;ThresholdBetween;VampireBodyDetector;VampireTorpedoesDetectorClose;VampireTorpedoesDetector;WhiteFilter;WhiteNoiseTakedown";
         // <FACTORY_GENERATOR_ITEMS_LIST/>
}

}  // namespace proc_image_processing
