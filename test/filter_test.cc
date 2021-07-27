#include <gtest/gtest.h>
#include <ros/ros.h>
#include "proc_image_processing/cpu/server/global_param_handler.h"
#include "proc_image_processing/cpu/filters/filter.h"
#include "proc_image_processing/cpu/server/filter_factory.h"

TEST(FilterTest, test) {
    proc_image_processing::GlobalParamHandler globalParamHandler;
    std::unique_ptr<proc_image_processing::Filter> accumulatorFilter = proc_image_processing::FilterFactory::createInstance(
            "AccumulatorFilter", globalParamHandler);
    ASSERT_EQ(accumulatorFilter->getName(), "AccumulatorFilter");
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
