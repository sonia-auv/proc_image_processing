#include <gtest/gtest.h>
#include "proc_image_processing/cpu/server/filter_factory.h"
#include "proc_image_processing/cpu/server/global_parameter_handler.h"

TEST(FilterFactoryTest, TestCreateInstance) {
    proc_image_processing::GlobalParameterHandler handler;
    std::shared_ptr<proc_image_processing::Filter> f;
    ros::NodeHandlePtr nhp;

    f = proc_image_processing::FilterFactory::createInstance("AccumulatorFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::AccumulatorFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("AdaptiveThresholdFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::AdaptiveThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("BackgroundSubtractFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::BackgroundSubtractFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("BilateralFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::BilateralFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("BlurFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::BlurFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CannyFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::CannyFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CenterDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::CenterDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CLAHEFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::CLAHEFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ContrastAndBrightnessFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::ContrastAndBrightnessFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ConvexHullFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::ConvexHullFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CropFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::CropFilter *>(f.get()), nullptr);

    // TODO breaks the CI (hangs)
    // f = proc_image_processing::FilterFactory::createInstance("Deep2019Filter", handler, nhp);
    // ASSERT_NE(dynamic_cast<proc_image_processing::Deep2019Filter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("DilateFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::DilateFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("EqualizeHistogramFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::EqualizeHistogramFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ErodeFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::ErodeFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("FenceDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::FenceDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("GateDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::GateDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HandleDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::HandleDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HideSubmarineFrameFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::HideSubmarineFrameFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HoughLineFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::HoughLineFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HSVThresholdFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::HSVThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("InRangeFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::InRangeFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("IntervalThresholdFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::IntervalThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("LaplacianFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::LaplacianFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("MissionTestFakeStringFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::MissionTestFakeStringFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("MorphologyFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::MorphologyFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("OriginalImageFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::OriginalImageFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("PipeAngleDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::PipeAngleDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("RemoveMaskFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::RemoveMaskFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("RotateFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::RotateFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ScharrAddingFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::ScharrAddingFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ScharrFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::ScharrFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SobelFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::SobelFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SquareDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::SquareDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("StatisticalThresholdFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::StatisticalThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SubtractAllPlanesFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::SubtractAllPlanesFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SubtractPlaneAdderFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::SubtractPlaneAdderFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("TestFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::TestFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ThresholdFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::ThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("VampireBodyDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::VampireBodyDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("VampireTorpedoesCloseDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::VampireTorpedoesCloseDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("VampireTorpedoesDetector", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::VampireTorpedoesDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("WhiteFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::WhiteFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("WhiteNoiseRemovalFilter", handler, nhp);
    ASSERT_NE(dynamic_cast<proc_image_processing::WhiteNoiseRemovalFilter *>(f.get()), nullptr);
}

TEST(FilterFactoryTest, TestGetFilters) {
    std::vector<std::string> expectedFilters{
            "AccumulatorFilter",
            "AdaptiveThresholdFilter",
            "BackgroundSubtractFilter",
            "BilateralFilter",
            "BlurFilter",
            "CannyFilter",
            "CenterDetector",
            "CLAHEFilter",
            "ContrastAndBrightnessFilter",
            "ConvexHullFilter",
            "CropFilter",
            // "Deep2019Filter",
            "DilateFilter",
            "EqualizeHistogramFilter",
            "ErodeFilter",
            "FenceDetector",
            "GateDetector",
            "HandleDetector",
            "HideSubmarineFrameFilter",
            "HoughLineFilter",
            "HSVThresholdFilter",
            "InRangeFilter",
            "IntervalThresholdFilter",
            "LaplacianFilter",
            "MissionTestFakeStringFilter",
            "MorphologyFilter",
            "OriginalImageFilter",
            "PipeAngleDetector",
            "RemoveMaskFilter",
            "RotateFilter",
            "ScharrAddingFilter",
            "ScharrFilter",
            "SobelFilter",
            "SquareDetector",
            "StatisticalThresholdFilter",
            "SubtractAllPlanesFilter",
            "SubtractPlaneAdderFilter",
            "TestFilter",
            "ThresholdFilter",
            "VampireBodyDetector",
            "VampireTorpedoesCloseDetector",
            "VampireTorpedoesDetector",
            "WhiteFilter",
            "WhiteNoiseRemovalFilter"
    };


    std::stringstream stream(proc_image_processing::FilterFactory::getFilters());
    std::set<std::string> actualFilters;
    while (stream.good()) {
        std::string s;
        getline(stream, s, ';');
        actualFilters.emplace(s);
    }

    for (auto &f:expectedFilters) {
        ASSERT_TRUE(actualFilters.find(f) != actualFilters.end());
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
