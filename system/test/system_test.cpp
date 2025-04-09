#include "system/HelperBlocks.hpp"
#include "system/System.hpp"
#include <gtest/gtest.h>

using namespace KinovaRobustControl::System;

TEST(SystemTest, BasicTest)
{
    KinovaRobustControl::System::System sys("basic test system");
    auto const_block =
        ConstantBlock<std::string>::make_shared("string const", "aba", 0.2, 10);
    auto mock_block = MockingBlock<std::string>::make_shared("mocking block");
    ASSERT_FALSE(sys.check_block("string const"));
    // add
    sys.add_block(const_block);
    sys.add_block(mock_block);
    ASSERT_TRUE(sys.check_block("string const"));
    ASSERT_TRUE(sys.check_block("mocking block"));
    // connect
    sys.connect_blocks(const_block, mock_block);
    // sys.connect_blocks("string const", "mocking block"); // also works
    sys.set_verbose().set_timeout(5).run();
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
