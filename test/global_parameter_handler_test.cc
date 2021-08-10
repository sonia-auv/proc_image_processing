#include <gtest/gtest.h>
#include "proc_image_processing/cpu/server/global_parameter_handler.h"
#include "proc_image_processing/cpu/server/parameter_interface.h"
#include "proc_image_processing/cpu/server/parameter.h"
#include "proc_image_processing/cpu/server/target.h"

TEST(GlobalParameterHandlerTest, TestParameters) {
    proc_image_processing::GlobalParameterHandler gph;
    std::vector<proc_image_processing::ParameterInterface *> v1;
    std::vector<proc_image_processing::ParameterInterface *> v2;

    proc_image_processing::Parameter<int> expected1("param1", 23, &v1, "Param 1 description");
    ASSERT_EQ(v1.size(), 1);
    proc_image_processing::Parameter<int> expected2("param1", 3, &v2, "Param 1 duplicate description");
    ASSERT_EQ(v1.size(), 1);

    gph.addParameter(&expected1);
    ASSERT_THROW(gph.addParameter(&expected2), std::invalid_argument);
    try {
        gph.addParameter(&expected2);
    } catch (std::invalid_argument &e) {
        ASSERT_EQ(std::string(e.what()), "A parameter with the same name already exists!");
    }

    auto actual = gph.getParameter("param1");
    ASSERT_EQ(actual->getName(), expected1.getName());
    ASSERT_EQ(actual->getType(), expected1.getType());
    ASSERT_EQ(actual->getStringValue(), expected1.getStringValue());
    ASSERT_EQ(actual->getDescription(), expected1.getDescription());

    std::vector<proc_image_processing::ParameterInterface *> v3;
    proc_image_processing::Parameter<int> expected3("param2", 6, &v3, "Param 2 description");
    gph.addParameter(&expected3);
    gph.removeParameter("param1");
    ASSERT_EQ(gph.getParameter("param1"), nullptr);
    actual = gph.getParameter("param2");
    ASSERT_EQ(actual->getName(), expected3.getName());
}

TEST(GlobalParameterHandlerTest, TestTargets) {
    proc_image_processing::GlobalParameterHandler gph;
    proc_image_processing::Target expected("header", 1, 1, 1, 1, 1, 1, 1, "spec1", "spec2");
    gph.addTarget(expected);
    std::queue<proc_image_processing::Target> targets = gph.getTargetQueue();
    ASSERT_EQ(targets.size(), 1);
    auto actual = targets.front();
    ASSERT_EQ(actual.getAngle(), expected.getAngle());
    ASSERT_EQ(actual.getCenter(), expected.getCenter());
    ASSERT_EQ(actual.getDimension(), expected.getDimension());
    ASSERT_EQ(actual.getHeader(), expected.getHeader());
    ASSERT_EQ(actual.getSpecialField1(), expected.getSpecialField1());
    ASSERT_EQ(actual.getSpecialField2(), expected.getSpecialField2());

    gph.addTarget(expected);
    targets = gph.getTargetQueue();
    ASSERT_EQ(targets.size(), 2);
    gph.clearTarget();
    targets = gph.getTargetQueue();
    ASSERT_EQ(targets.size(), 0);
}

TEST(GlobalParameterHandler, TestImage) {
    proc_image_processing::GlobalParameterHandler gph;
    cv::Mat image(4, 4, CV_8UC1);
    cv::randu(image, cv::Scalar(0), cv::Scalar(255));
    gph.setOriginalImage(image);
    auto actual = gph.getOriginalImage();
    cv::Mat diff;
    cv::compare(actual, image, diff, cv::CmpTypes::CMP_EQ);
    ASSERT_FALSE(cv::countNonZero(diff) == 0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
