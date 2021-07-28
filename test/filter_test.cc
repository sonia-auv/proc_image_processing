#include <gtest/gtest.h>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "proc_image_processing/cpu/server/global_param_handler.h"
#include "proc_image_processing/cpu/filters/filter.h"

class MockFilter : public proc_image_processing::Filter {
public:
    using Ptr = std::shared_ptr<MockFilter>;

    explicit MockFilter(const proc_image_processing::GlobalParamHandler &handler) : proc_image_processing::Filter(
            handler) {
        setName("MockFilter");
    }

    ~MockFilter() override = default;

    void apply(cv::Mat &image) override {

    }
};

TEST(FilterTest, test) {
    proc_image_processing::GlobalParamHandler handler;
    std::unique_ptr<proc_image_processing::Filter> f = std::move(std::make_unique<MockFilter>(handler));
    ASSERT_EQ(f->getName(), "MockFilter");

    std::string expectedName = "MockFilterModified";
    f->setName(expectedName);
    ASSERT_EQ(f->getName(), expectedName);

    // == Integer params
    // Test min/max limits
    f->addGlobalParameter("param1", 0, 0, 4);
    ASSERT_EQ(f->getParameterValue("param1"), "param1|Integer|0|0|4|");
    f->addGlobalParameter("param2", 4, 0, 4);
    ASSERT_EQ(f->getParameterValue("param2"), "param2|Integer|4|0|4|");

    // Test min/max errors
    ASSERT_THROW(f->addGlobalParameter("badParam", 0, 1, 3), std::invalid_argument);
    try {
        f->addGlobalParameter("badParam", 0, 1, 3);
    } catch (std::invalid_argument &e) {
        ASSERT_EQ(*e.what(), *"Value can't be less than minimum!");
    }

    ASSERT_THROW(f->addGlobalParameter("badParam", 4, 1, 3), std::invalid_argument);
    try {
        f->addGlobalParameter("badParam", 4, 1, 3);
    } catch (std::invalid_argument &e) {
        ASSERT_EQ(*e.what(), *"Value can't be more than maximum!");
    }

    // == Double params
    // Test min/max limits
    f->addGlobalParameter("param3", 0.01, 0.01, 3.4);
    ASSERT_EQ(f->getParameterValue("param3"), "param3|Double|0.010000|0.010000|3.400000|");
    f->addGlobalParameter("param4", 3.4, 0.01, 3.4);
    ASSERT_EQ(f->getParameterValue("param4"), "param4|Double|3.400000|0.010000|3.400000|");

    // Test min/max errors
    ASSERT_THROW(f->addGlobalParameter("badParam", 0.009, 0.01, 3.4), std::invalid_argument);
    try {
        f->addGlobalParameter("badParam", 0.009, 0.01, 3.4);
    } catch (std::invalid_argument &e) {
        ASSERT_EQ(*e.what(), *"Value can't be less than minimum!");
    }

    ASSERT_THROW(f->addGlobalParameter("badParam", 3.41, 0.01, 3.4), std::invalid_argument);
    try {
        f->addGlobalParameter("badParam", 3.41, 0.01, 3.4);
    } catch (std::invalid_argument &e) {
        ASSERT_EQ(*e.what(), *"Value can't be more than maximum!");
    }

    // == Boolean params
    f->addGlobalParameter("param5", true);
    ASSERT_EQ(f->getParameterValue("param5"), "param5|Boolean|1|||");
    f->addGlobalParameter("param6", false);
    ASSERT_EQ(f->getParameterValue("param6"), "param6|Boolean|0|||");


    // == String params
//    f->addGlobalParameter("param7", "stringValue1");
//    ASSERT_EQ(f->getParameterValue("param7"), "param7|String|stringValue1|||");
//    f->addGlobalParameter("param8", "stringValue2");
//    ASSERT_EQ(f->getParameterValue("param8"), "param8|String|stringValue2|||");
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "proc_image_processing");
    return RUN_ALL_TESTS();
}
