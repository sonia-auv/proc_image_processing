#include <gtest/gtest.h>
#include "ros/ros.h"
#include "proc_image_processing/cpu/server/filter_factory.h"

TEST(FilterFactoryTest, TestCreateInstance) {
    proc_image_processing::GlobalParameterHandler handler;
    std::shared_ptr<proc_image_processing::Filter> f;

    f = proc_image_processing::FilterFactory::createInstance("AccumulatorFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::AccumulatorFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("AdaptiveThresholdFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::AdaptiveThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("BackgroundSubtractFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::BackgroundSubtractFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("BilateralFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::BilateralFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("BlurFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::BlurFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CannyFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::CannyFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CenterCoffinDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::CenterCoffinDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CLAHEFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::CLAHEFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ContrastAndBrightnessFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::ContrastAndBrightnessFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ConvexHullFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::ConvexHullFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("CropFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::CropFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("Deep2019Filter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::Deep2019Filter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("DilateFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::DilateFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("EqualizeHistogramFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::EqualizeHistogramFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ErodeFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::ErodeFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("FenceDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::FenceDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("GateDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::GateDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HandleDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::HandleDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HideSubmarineFrameFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::HideSubmarineFrameFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HoughLineFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::HoughLineFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("HSVThresholdFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::HSVThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("InRangeFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::InRangeFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("IntervalThresholdFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::IntervalThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("LaplacianFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::LaplacianFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("MissionTestFakeStringFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::MissionTestFakeStringFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("MorphologyFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::MorphologyFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("OriginalImageFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::OriginalImageFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("PipeAngleDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::PipeAngleDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("RemoveMaskFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::RemoveMaskFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("RotateFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::RotateFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ScharrAddingFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::ScharrAddingFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ScharrFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::ScharrFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SobelFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::SobelFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SquareDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::SquareDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("StatisticalThresholdFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::StatisticalThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SubtractAllPlanesFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::SubtractAllPlanesFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("SubtractPlaneAdderFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::SubtractPlaneAdderFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("TestFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::TestFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("ThresholdFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::ThresholdFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("VampireBodyDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::VampireBodyDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("VampireTorpedoesCloseDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::VampireTorpedoesCloseDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("VampireTorpedoesDetector", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::VampireTorpedoesDetector *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("WhiteFilter", handler);
    ASSERT_NE(dynamic_cast<proc_image_processing::WhiteFilter *>(f.get()), nullptr);

    f = proc_image_processing::FilterFactory::createInstance("WhiteNoiseRemovalFilter", handler);
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
            "CenterCoffinDetector",
            "CLAHEFilter",
            "ContrastAndBrightnessFilter",
            "ConvexHullFilter",
            "CropFilter",
            "Deep2019Filter",
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


    std::string filtersString = proc_image_processing::FilterFactory::getFilters();
    std::stringstream stream(filtersString);
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
