#include <gtest/gtest.h>
#include "opencv2/opencv.hpp"
#include "proc_image_processing/cpu/server/global_parameter_handler.h"
#include "proc_image_processing/cpu/filters/filter.h"

class MockFilter : public proc_image_processing::Filter {
public:
    using Ptr = std::shared_ptr<MockFilter>;

    explicit MockFilter(const proc_image_processing::GlobalParameterHandler &handler) : proc_image_processing::Filter(
            handler) {
        setName("MockFilter");
    }

    ~MockFilter() override = default;

    void apply(cv::Mat &image) override {
        cv::threshold(image, image, 128.0, 255.0, CV_8UC1);
    }
};

// TEST(FilterTest, TestNotify){
// TODO
// proc_image_processing::GlobalParamHandler handler;
// auto f = std::move(std::make_unique<MockFilter>(handler));
// }

TEST(FilterTest, TestBaseFeatures) {
    proc_image_processing::GlobalParameterHandler handler;
    auto f = std::move(std::make_unique<MockFilter>(handler));

    // Test name
    ASSERT_EQ(f->getName(), "MockFilter");
    std::string expectedName = "MockFilterModified";
    f->setName(expectedName);
    ASSERT_EQ(f->getName(), expectedName);
}

TEST(FilterTest, TestApply) {
    proc_image_processing::GlobalParameterHandler handler;
    auto f = std::move(std::make_unique<MockFilter>(handler));

    cv::Mat in(4, 4, CV_8UC1);
    cv::randu(in, cv::Scalar(0), cv::Scalar(255));
    cv::Mat before;
    in.copyTo(before);

    f->apply(in);

    cv::Mat diff;
    cv::compare(in, before, diff, cv::CmpTypes::CMP_EQ);
    ASSERT_TRUE(cv::countNonZero(diff) == 0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
