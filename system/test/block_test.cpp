#include "system/BaseBlock.hpp"
#include "system/HelperBlocks.hpp"
#include "system/ROSHelper.hpp"
#include "system/System.hpp"
#include "std_msgs/msg/string.hpp"
#include <gtest/gtest.h>
#include <std_msgs/msg/detail/string__struct.hpp>

using namespace KinovaRobustControl::System;
TEST(BlockTest, BaseBlockTest)
{
    auto const_block =
        ConstantBlock<std::string>::make_shared("string const", "aba", 0.2, 10);
    std::cout << const_block->describe_connection() << std::endl;
    ASSERT_FALSE(const_block->has_input("aba"));
    ASSERT_EQ(const_block->get_block_name(), "string const");
}

TEST(BlockTest, ROSBlockTest)
{
    using std_msgs::msg::String;
    auto node = std::make_shared<rclcpp::Node>("test_node");
    /// create subscriber block that converts ros message string to std::string
    auto sub_blk = SubscriberBlock<String, std::string>::make_shared(
        "Subscriber Block", "test_topic", node,
        [](const String &in) { return std::string(in.data); });

    /// mocking block that will output what subscriber received
    auto mock_blk = MockingBlock<std::string>::make_shared("mblk");

    /// publisher block
    auto pub_blk = PublisherBlock<String>::make_shared("Publisher Block",
                                                       "test_topic", node);
    /// constant block that provides message to publisher
    std_msgs::msg::String str;
    str.data = "aba";
    auto const_block = ConstantBlock<std_msgs::msg::String>::make_shared(
        "str_const", str, 0.2);

    System sys("ROS System");

    sys.connect_blocks(const_block, pub_blk)
        .connect_blocks(sub_blk, mock_blk)
        .set_verbose()
        .set_timeout(5);

    sys.run();
}

TEST(BlockTest, HelperBlocksTest)
{
    auto const_block =
        ConstantBlock<std::string>::make_shared("str_cst", "aba", 0.2, 10);
    auto mock_block = MockingBlock<std::string>::make_shared("mocking block");
    // run with timeout
    System("HelperBlockSystem")
        .connect_blocks(const_block, mock_block)
        .set_verbose()
        .set_timeout(5)
        .run();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
