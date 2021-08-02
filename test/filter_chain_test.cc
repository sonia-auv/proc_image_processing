#include <gtest/gtest.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include "proc_image_processing/cpu/server/filter_chain.h"
#include "proc_image_processing/cpu/config.h"

TEST(FilterChainTest, BaseFeatures) {
    proc_image_processing::FilterChain fc1("filter_chain_test_1");
    ASSERT_EQ(fc1.getName(), "filter_chain_test_1");

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


    proc_image_processing::FilterChain fc2("filter_chain_test_2");
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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
