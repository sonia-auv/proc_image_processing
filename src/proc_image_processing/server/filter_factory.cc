/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include "proc_image_processing/server/filter_factory.h"

namespace proc_image_processing {
Filter *FilterFactory::createInstance(const std::string &name,
                                      const GlobalParamHandler &globalParams) {
  if (name == "Blurr") {
    return new Blurr(globalParams);
  } else if (name == "Dilate") {
    return new Dilate(globalParams);
  } else if (name == "Erode") {
    return new Erode(globalParams);
  } else if (name == "MissionTestFakeString") {
    return new MissionTestFakeString(globalParams);
  } else if (name == "TestFilter") {
    return new TestFilter(globalParams);
  } else if (name == "Morphology") {
    return new Morphology(globalParams);
  } else if (name == "OriginalImage") {
    return new OriginalImage(globalParams);
  } else if (name == "Scharr") {
    return new Scharr(globalParams);
  } else if (name == "ScharrAdding") {
    return new ScharrAdding(globalParams);
  } else if (name == "StatsThreshold") {
    return new StatsThreshold(globalParams);
  } else if (name == "SubtractAllPlanes") {
    return new SubtractAllPlanes(globalParams);
  } else if (name == "Threshold") {
    return new Threshold(globalParams);
  } else if (name == "Rotate") {
    return new Rotate(globalParams);
  } else if (name == "FenceDetector") {
    return new FenceDetector(globalParams);
  } else if (name == "ImageAccumulator") {
    return new ImageAccumulator(globalParams);
  } else if (name == "Sobel") {
    return new Sobel(globalParams);
  } else if (name == "SubmarineFrameMasker") {
    return new SubmarineFrameMasker(globalParams);
  } else if (name == "InRange") {
    return new InRange(globalParams);
  } else if (name == "ConvexHull") {
    return new ConvexHull(globalParams);
  } else if (name == "Laplacian") {
    return new Laplacian(globalParams);
  } else if (name == "Canny") {
    return new Canny(globalParams);
  } else if (name == "HoughLine") {
    return new HoughLine(globalParams);
  } else if (name == "AdaptiveThreshold") {
    return new AdaptiveThreshold(globalParams);
  } else if (name == "HandleDetector") {
    return new HandleDetector(globalParams);
  } else if (name == "WhiteNoiseTakedown") {
    return new WhiteNoiseTakedown(globalParams);
  } else if (name == "BilateralFilter") {
    return new BilateralFilter(globalParams);
  } else if (name == "BackgroundSubstract") {
    return new BackgroundSubstract(globalParams);
  } else if (name == "ImageCropper") {
    return new ImageCropper(globalParams);
  } else if (name == "GateDetector") {
    return new GateDetector(globalParams);
  } else if (name == "RemoveMask") {
    return new RemoveMask(globalParams);
  } else if (name == "Equalize") {
    return new Equalize(globalParams);
  } else if (name == "HSVThreshold") {
    return new HSVThreshold(globalParams);
  } else if (name == "ContrastBrightness") {
    return new ContrastBrightness(globalParams);
  } else if (name == "SquareDetection") {
    return new SquareDetection(globalParams);
  } else if (name == "WhiteFilter") {
    return new WhiteFilter(globalParams);
  } else if (name == "PipeAngleDetector") {
    return new PipeAngleDetector(globalParams);
  } else if (name == "Deep2019") {
    return new Deep2019(globalParams);
  } else if (name == "VampireTorpedoesDetector") {
    return new VampireTorpedoesDetector(globalParams);
  } else if (name == "VampireBodyDetector") {
    return new VampireBodyDetector(globalParams);
  } else if (name == "VampireTorpedoesDetectorClose") {
    return new VampireTorpedoesDetectorClose(globalParams);
  } else if (name == "ThresholdBetween") {
    return new ThresholdBetween(globalParams);
  } else if (name == "CenterCoffinDetector") {
    return new CenterCoffinDetector(globalParams);
  } else {
    return nullptr;
  }
}

std::string FilterFactory::GetFilterList() {
  return "Blurr;Dilate;Erode;MissionTestFakeString;TestFilter;"
         "Morphology;OriginalImage;Scharr;ScharrAdding;"
         "StatsThreshold;SubtractAllPlanes;Threshold;Rotate;"
         "FenceDetector;ImageAccumulator;"
         "Sobel;SubmarineFrameMasker;InRange;ConvexHull;"
         "Laplacian;Canny;HoughLine;AdaptiveThreshold;"
         "HandleDetector;WhiteNoiseTakedown;BilateralFilter;"
         "BackgroundSubstract;ImageCropper;GateDetector;RemoveMask;"
         "HSVThreshold;ContrastBrightness;Equalize;SquareDetection;WhiteFilter;"
         "PipeAngleDetector;Deep2019;VampireTorpedoesDetector;VampireBodyDetector;"
         "VampireTorpedoesDetectorClose;ThresholdBetween;CenterCoffinDetector";
}

}  // namespace proc_image_processing
