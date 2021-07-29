#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "proc_image_processing/cpu/server/filter_chain.h"
#include "proc_image_processing/cpu/config.h"

class MockFilterChain : public proc_image_processing::FilterChain {
public:
    explicit MockFilterChain(const std::string &name) : proc_image_processing::FilterChain(name),
                                                        filepath_(std::string("assets/config/filterchain") + name +
                                                                  proc_image_processing::kFilterChainExt),
                                                        name_(name),
                                                        param_handler_(),
                                                        observer_index_(0) {
        deserialize();
        observer_index_ = filters_.size() - 1;
    }
};

TEST(FilterChainTest, BaseFeatures
) {
MockFilterChain fc("fc1");
ASSERT_EQ(fc
.

getName(),

"fc1");
ASSERT_EQ(fc
.

getFilters()

.

size(),

0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
