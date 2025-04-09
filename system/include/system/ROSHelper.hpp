/**
 * @file
 * @brief helper classes related to ROS
 **/
#pragma once
#include "system/BaseBlock.hpp"
#include "system/System.hpp"
#include <rclcpp/rclcpp.hpp>
namespace KinovaRobustControl
{
namespace System
{
/**
 * @brief default conversion between two types */
template <typename InT, typename OutT> struct default_convert
{
    OutT operator()(const InT &t)
    {
        return OutT(t);
    }
};

/**
 * @brief Block for receiving ROS2 topic message
 * @param SubT: ROS Topic Message type
 * @param OutT: output type
 * @param Covert: functor used to convert SubT to OutT
 **/
template <typename SubT, typename OutT = SubT>
class SubscriberBlock : public BaseBlock
{
  public:
    using Convert = std::function<OutT(const SubT &)>;
    /**
     * @brief block maker function
     * @param block_name: name of the block
     * @param topic_name: name of topic to subscribe
     * @param node: ros node to use
     * @param conv: conversion function
     * @return %shared_ptr to the created block
     **/
    static BaseBlock::SharedPtr make_shared(
        const std::string &block_name, const std::string &topic_name,
        rclcpp::Node::SharedPtr node,
        Convert &&conv = default_convert<SubT, OutT>())
    {
        auto output_port{OutputPort<OutT>::make_shared()};
        return BaseBlock::SharedPtr(
            new SubscriberBlock(block_name, topic_name, output_port, node,
                                std::forward<Convert>(conv)));
    }

    /**
     * @brief listening on topic and update message
     **/
    virtual void run() override
    {
        auto sub = node->create_subscription<SubT>(
            topic_name, 10,
            std::bind(&SubscriberBlock<SubT, OutT>::sub_callback, this,
                      std::placeholders::_1));
        while (true)
        {
            check_stop();
            rclcpp::spin_some(this->node);
        }
    }

    /**
     * @brief subscriber callback function for ROS
     * @param msg: message to receive
     **/
    void sub_callback(const std::shared_ptr<SubT> msg)
    {
        default_output->set(convert(*msg));
    }

  private:
    /**
     * @brief default constructor
     * @param block_name: name of the block
     * @param output: output port to use
     * @param node: ros2 node
     **/
    SubscriberBlock(const std::string &block_name,
                    const std::string &topic_name,
                    AbstractOutputPort::SharedPtr output,
                    rclcpp::Node::SharedPtr node, Convert &&conv)
        : BaseBlock(block_name, output), node(node), topic_name(topic_name),
          convert(std::forward<Convert>(conv))
    {
    }

  private:
    rclcpp::Node::SharedPtr node;
    const std::string topic_name;
    Convert convert;
};

/**
 * @brief helper block for publishing ros message
 * @warning the output block hasn't been used here
 **/
template <typename InT, typename PubT = InT>
class PublisherBlock : public BaseBlock
{
  public:
    using Convert = std::function<PubT(const InT &)>;
    /**
     * @brief block maker function
     * @param block_name: name to the block
     * @param topic_name: name to the topic
     * @return %shared_ptr to the created block
     **/
    static BaseBlock::SharedPtr make_shared(
        const std::string &block_name, const std::string &topic_name,
        rclcpp::Node::SharedPtr node,
        Convert &&conv = default_convert<InT, PubT>())
    {
        return BaseBlock::SharedPtr(new PublisherBlock(
            block_name, topic_name, node, std::forward<Convert>(conv)));
    }

    /**
     * @brief listening on topic and update message
     **/
    virtual void run() override
    {
        if (!input->is_connected())
        {
            const char *err_msg = "Input Port not connected";
            logger.critical(err_msg);
            throw std::runtime_error(err_msg);
        }

        while (true)
        {
            auto val_ptr = input->get_new<InT>();
            pub->publish(convert(*val_ptr));
        }
    }

  private:
    /**
     * @brief default constructor
     * @param block_name: name of the block
     * @param topic_name: name of the topic
     * @param node: ros2 node
     **/
    PublisherBlock(const std::string &block_name, const std::string &topic_name,
                   rclcpp::Node::SharedPtr node, Convert &&conv)
        : BaseBlock(block_name, OutputPort<bool>::make_shared()), node(node),
          pub(node->create_publisher<PubT>(topic_name, 10)),
          input(register_input<InT>("input")),
          convert(std::forward<Convert>(conv))
    {
    }

  private:
    rclcpp::Node::SharedPtr node;
    typename rclcpp::Publisher<PubT>::SharedPtr pub;
    AbstractInputPort::SharedPtr input;
    Convert convert;
};

/**
 * @brief combination of system and ROS2 node
 **/
class NodeSystem : public System, public rclcpp::Node
{
  public:
    /**
     * @brief maker
     **/
    static std::shared_ptr<NodeSystem> make(const std::string &system_name)
    {
        return std::shared_ptr<NodeSystem>(new NodeSystem(system_name));
    }

  private:
    /** @brief ctor */
    NodeSystem(const std::string &system_name)
        : System(system_name), Node(system_name)
    {
    }
};

} // namespace System
} // namespace KinovaRobustControl
