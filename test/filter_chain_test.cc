#include <gtest/gtest.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include "proc_image_processing/cpu/server/filter_chain.h"
#include "proc_image_processing/cpu/config.h"

TEST(FilterChainTest, TestBaseFeatures) {
    proc_image_processing::FilterChain fc1("filter_chain_test_1");
    ASSERT_EQ(fc1.getName(), "filter_chain_test_1");
    fc1.setName("new_filter_chain_name");
    ASSERT_EQ(fc1.getName(), "new_filter_chain_name");

    // Test expected filters
    std::set<std::string> expectedFilters{"CropFilter", "ThresholdFilter", "CLAHEFilter"};
    std::vector<proc_image_processing::Filter::Ptr> actualFilters = fc1.getFilters();
    ASSERT_EQ(actualFilters.size(), expectedFilters.size());
    for (const auto &filter: actualFilters) {
        ASSERT_TRUE(expectedFilters.find(filter->getName()) != expectedFilters.end());
    }

    // Test filter order
    ASSERT_EQ(fc1.getFilter(size_t(0))->getName(), "CropFilter");
    ASSERT_EQ(fc1.getFilter(size_t(1))->getName(), "ThresholdFilter");
    ASSERT_EQ(fc1.getFilter(size_t(2))->getName(), "CLAHEFilter");

    // Test bad range
    ASSERT_EQ(fc1.getFilter(size_t(-1)), nullptr);
    ASSERT_EQ(fc1.getFilter(size_t(3)), nullptr);

    proc_image_processing::FilterChain fc2("filter_chain_test_2");
    ASSERT_EQ(fc2.getName(), "filter_chain_test_2");
    ASSERT_EQ(fc2.getFilters().size(), 7);

    std::map<int, proc_image_processing::Filter::Ptr> actualSameFilters = fc2.getFilters("BlurFilter");
    ASSERT_EQ(actualSameFilters.size(), 2);
    std::shared_ptr<proc_image_processing::Filter> bf1 = actualSameFilters[1];
    std::shared_ptr<proc_image_processing::Filter> bf2 = actualSameFilters[3];

    ASSERT_EQ(bf1->getParameterValue("Type"), "Type|Integer|1|0|3|1=Blur, 2=GaussianBlur, 3=MedianBlur");
    ASSERT_EQ(bf1->getParameterValue("Kernel size"), "Kernel size|Integer|12|0|35|");
    ASSERT_EQ(bf2->getParameterValue("Type"), "Type|Integer|2|0|3|1=Blur, 2=GaussianBlur, 3=MedianBlur");
    ASSERT_EQ(bf2->getParameterValue("Kernel size"), "Kernel size|Integer|23|0|35|");

    ASSERT_EQ(fc2.getFilters("ConvexHullFilter").size(), 3);
}

TEST(FilterChainTest, TestFilters) {
    proc_image_processing::FilterChain fc("filter_chain_test_3");

    // Try with a bad filter
    ASSERT_THROW(fc.addFilter("bad_filter"), std::invalid_argument);
    try {
        fc.addFilter("bad_filter");
    } catch (const std::invalid_argument &e) {
        ASSERT_EQ(std::string(e.what()), "Filter bad_filter does not exist!");
    }

    auto initialSize = fc.getFilters().size();

    // Use a good filter
    fc.addFilter("CLAHEFilter");
    auto actualLastFilter = fc.getFilter(fc.getFilters().size() - 1);

    // Should be last
    ASSERT_EQ(initialSize + 1, fc.getFilters().size());
    ASSERT_EQ(actualLastFilter->getName(), "CLAHEFilter");


    auto lastFilter = fc.getFilters().size() - 1;

    // Should not be able to move down
    ASSERT_THROW(fc.moveFilterDown(lastFilter), std::invalid_argument);
    try {
        fc.moveFilterDown(lastFilter);
    } catch (const std::invalid_argument &e) {
        ASSERT_EQ(std::string(e.what()), "Cannot move filter down, it is already at the end of the filter chain!");
    }

    // Should not be able to move up
    ASSERT_THROW(fc.moveFilterUp(0), std::invalid_argument);
    try {
        fc.moveFilterUp(0);
    } catch (const std::invalid_argument &e) {
        ASSERT_EQ(std::string(e.what()), "Cannot move filter up, it is already at the beginning of the filter chain!");
    }

    fc.moveFilterDown(lastFilter - 1);
    // Should have moved down
    ASSERT_EQ(fc.getFilter(lastFilter)->getName(), "EqualizeHistogramFilter");
    ASSERT_EQ(fc.getFilter(lastFilter - 1)->getName(), "CLAHEFilter");

    fc.moveFilterUp(lastFilter);
    // Should have moved up
    ASSERT_EQ(fc.getFilter(lastFilter)->getName(), "CLAHEFilter");
    ASSERT_EQ(fc.getFilter(lastFilter - 1)->getName(), "EqualizeHistogramFilter");

    fc.moveFilterUp(1);
    // Should have moved up
    ASSERT_EQ(fc.getFilter(0)->getName(), "BlurFilter");
    ASSERT_EQ(fc.getFilter(1)->getName(), "BilateralFilter");

    fc.moveFilterDown(0);
    // Should have moved up
    ASSERT_EQ(fc.getFilter(0)->getName(), "BilateralFilter");
    ASSERT_EQ(fc.getFilter(1)->getName(), "BlurFilter");

    // Test contains filter
    ASSERT_TRUE(fc.containsFilter("BilateralFilter"));
    ASSERT_TRUE(fc.containsFilter("BlurFilter"));
    ASSERT_TRUE(fc.containsFilter("ConvexHullFilter"));
    ASSERT_TRUE(fc.containsFilter("EqualizeHistogramFilter"));
    ASSERT_TRUE(fc.containsFilter("CLAHEFilter"));
    ASSERT_FALSE(fc.containsFilter("InRangeFilter"));

    // Should remove CLAHEFilter
    fc.removeFilter(lastFilter);
    ASSERT_FALSE(fc.containsFilter("CLAHEFilter"));
    ASSERT_EQ(fc.getFilters().size(), initialSize);
}

TEST(FilterChainTest, TestSerialization) {
    proc_image_processing::FilterChain fc("filter_chain_test_3");
    fc.setName("filter_chain_test_3_modified");
    fc.addFilter("CLAHEFilter");
    fc.setFilepath(
            proc_image_processing::kFilterChainPath + "/filter_chain_test_3_modified" +
            proc_image_processing::kFilterChainExt
    );
    ASSERT_TRUE(fc.serialize());

    // Overwrite current filter chain with written file
    ASSERT_TRUE(fc.deserialize());
    ASSERT_TRUE(fc.containsFilter("CLAHEFilter"));
    ASSERT_EQ(fc.getFilters().size(), 5);

    // Load the modified file (no overwrite)
    proc_image_processing::FilterChain fcModified("filter_chain_test_3_modified");
    ASSERT_TRUE(fcModified.containsFilter("CLAHEFilter"));
    ASSERT_EQ(fcModified.getFilters().size(), 5);
    fc.removeFilter(fc.getFilters().size() - 1);
    ASSERT_TRUE(fcModified.serialize());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
