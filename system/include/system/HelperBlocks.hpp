/**
 * @brief some handy blocks defined for simple usage/debug
 * @file
 **/
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "system/BaseBlock.hpp"
#include "system/Port/InputPort.hpp"
#include "system/Port/OutputPort.hpp"
#include <chrono>
#include <functional>
#include <ratio>

namespace KinovaRobustControl
{
namespace System
{
/**
 * @brief block that constantly setting a value
 **/
template <typename OutputT> class ConstantBlock final : public BaseBlock
{
  public:
    /**
     * @brief maker
     * @param block_name: name of block
     * @param val: value to send
     * @param interval: interval between sending (unit second)
     * @param cnt: time to output, 0 for infinity
     **/
    static BaseBlock::SharedPtr make_shared(const std::string &block_name,
                                            const OutputT &val,
                                            double interval = 0,
                                            std::size_t cnt = 0)
    {
        return BaseBlock::SharedPtr(
            new ConstantBlock(block_name, val, interval, cnt));
    }

    virtual void run() override
    {
        for (std::size_t i{0}; cnt == 0 || i < cnt; i++)
        {
            check_stop();
            default_output->set(val);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<std::size_t>(interval * 1000)));
        }
    }

  private:
    ConstantBlock(const std::string &block_name, const OutputT &val,
                  double interval, std::size_t cnt)
        : BaseBlock(block_name, OutputPort<OutputT>::make_shared()), val(val),
          interval(interval), cnt(cnt)
    {
    }

    const OutputT val;
    const double interval;
    const std::size_t cnt;
};

/**
 * @brief test block that keeps printing the value when receiving updates
 **/
template <typename T> class MockingBlock final : public BaseBlock
{
  public:
    /**
     * @brief maker
     **/
    static BaseBlock::SharedPtr make_shared(const std::string &name)
    {
        return BaseBlock::SharedPtr(new MockingBlock<T>(name));
    }

    virtual void run() override
    {
        // NOTE: no check_stop() because get_new will have checking
        while (true)
        {
            auto ptr = input->get_new<T>();
            std::size_t version = input->get_data_version();
            logger.info("version {}: {}", version, *ptr);
            default_output->set(*ptr);
        }
    }

  private:
    MockingBlock(const std::string &name)
        : BaseBlock(name, OutputPort<T>::make_shared()),
          input(register_input<T>("input"))
    {
    }

    AbstractInputPort::SharedPtr input;
};
} // namespace System
} // namespace KinovaRobustControl
