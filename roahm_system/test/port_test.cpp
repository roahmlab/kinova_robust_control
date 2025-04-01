#include "roahm_system/Port/InputPort.hpp"
#include "roahm_system/Port/OutputPort.hpp"
#include "roahm_system/Ports.hpp"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace Roahm::System;

/**
 * @brief test for basic apis of %OutputPort class
 **/
TEST(PortTest, BasicOutputPortTest)
{
    std::vector<std::shared_ptr<AbstractOutputPort>> ports;
    // pushing back different types
    ports.push_back(OutputPort<int>::make_shared());
    ports.push_back(OutputPort<double>::make_shared());
    ports.push_back(OutputPort<uint>::make_shared());

    // set
    ports[0]->set<int>(1);
    ports[1]->set<double>(1.0);
    ports[2]->set<uint>(10);

    // get_new
    std::shared_ptr<const uint> val{nullptr};
    std::size_t ver{0};
    ports[2]->get_new<uint>(val, ver);
    ASSERT_EQ(*val, 10);
    ASSERT_EQ(ver, 1);
    // get_nowait
    ports[2]->set<uint>(100);
    ports[2]->get_nowait<uint>(val, ver);
    ASSERT_EQ(*val, 100);
    ASSERT_EQ(ver, 2);

    // check data version
    ASSERT_EQ(ports[0]->query_data_version(), 1);

    // should finish
    ports[0]->wait_init();

    // NOTE: copy constructor, should fail
    // AbstractOutputPort aba(*ports[0]);
}

/**
 * @brief multi-thread test for %OutputPort
 **/
TEST(PortTest, ParallelOutputPortTest)
{
    using namespace std::chrono_literals;
    AbstractOutputPort::SharedPtr op{OutputPort<int>::make_shared()};

    // well-defined for multi-threading
    std::atomic_bool got{false};
    // setter
    auto func1 = [&op, &got] {
        std::this_thread::sleep_for(5ms);
        ASSERT_FALSE(got);
        op->set<int>(10);
        std::this_thread::sleep_for(5ms);
        ASSERT_TRUE(got);
        op->set<int>(100);
    };
    // getter
    auto func2 = [&op, &got] {
        std::shared_ptr<const int> val;
        std::size_t ver{0};
        op->get_new<int>(val, ver);
        ASSERT_EQ(*val, 10);
        ASSERT_EQ(ver, 1);
        got = true;
        op->get_nowait<int>(val, ver);
        ASSERT_EQ(*val, 10);
        op->get_new<int>(val, ver);
        ASSERT_EQ(*val, 100);
        ASSERT_EQ(ver, 2);
    };
    std::thread t2(func2);
    std::thread t1(func1);
    t1.join();
    t2.join();
}

TEST(PortTest, BasicInputPortTest)
{
    std::vector<std::shared_ptr<AbstractOutputPort>> output_ports;
    // pushing back different types
    output_ports.push_back(OutputPort<int>::make_shared());
    output_ports.push_back(OutputPort<double>::make_shared());

    std::vector<AbstractInputPort::SharedPtr> input_ports;
    input_ports.push_back(InputPort<int>::make_shared());
    input_ports.push_back(InputPort<double>::make_shared());

    // no connection behaviors
    ASSERT_FALSE(input_ports[0]->is_connected());
    ASSERT_THROW(input_ports[0]->get_nowait<int>(), std::runtime_error);
    ASSERT_THROW(input_ports[0]->get_new<int>(), std::runtime_error);
    ASSERT_THROW(input_ports[0]->wait_init(), std::runtime_error);
    ASSERT_THROW(input_ports[0]->is_initialized(), std::runtime_error);

    // connect
    ASSERT_THROW(input_ports[0]->connect(output_ports[1]), std::runtime_error);
    input_ports[0]->connect(output_ports[0]);
    input_ports[1]->connect(output_ports[1]);
    ASSERT_TRUE(input_ports[0]->is_connected());
    ASSERT_FALSE(input_ports[0]->is_initialized());
    // not initialized yet
    ASSERT_THROW(input_ports[0]->get_nowait<int>(), std::runtime_error);

    output_ports[0]->set<int>(10);
    output_ports[1]->set<double>(100.);
    auto i0 = input_ports[0]->get_nowait<int>();
    auto i1 = input_ports[1]->get_new<double>();
    ASSERT_EQ(*i0, 10);
    ASSERT_EQ(*i1, 100.);
    ASSERT_EQ(input_ports[0]->get_data_version(), 1);
    output_ports[0]->set<int>(100);
    auto i2 = input_ports[0]->get_new<int>();
    ASSERT_EQ(*i2, 100);

    // copy ctor, should fail
    // AbstractInputPort io(*input_ports[0]);
}

/**
 * @brief multi-thread test for %InputPort
 **/
TEST(PortTest, ParallelInputPortTest)
{
    using namespace std::chrono_literals;
    AbstractOutputPort::SharedPtr op{OutputPort<int>::make_shared()};
    AbstractInputPort::SharedPtr ip1{InputPort<int>::make_shared()};
    AbstractInputPort::SharedPtr ip2{InputPort<int>::make_shared()};
    // connect
    ip1->connect(op);
    ip2->connect(op);

    // well-defined for multi-threading
    std::atomic_bool got{false};
    // setter
    auto setter = [&op, &got] {
        std::this_thread::sleep_for(5ms);
        ASSERT_FALSE(got);
        op->set<int>(10);
        std::this_thread::sleep_for(5ms);
        ASSERT_TRUE(got);
        op->set<int>(100);
    };
    // getter
    auto getter1 = [&ip1, &got] {
        ip1->wait_init();
        ASSERT_TRUE(ip1->is_initialized());
        auto i1 = ip1->get_nowait<int>();
        got = true;
        ASSERT_EQ(*i1, 10);
        auto i2 = ip1->get_nowait<int>();
        ASSERT_EQ(*i2, 10);
        auto i3 = ip1->get_new<int>();
        ASSERT_EQ(*i3, 100);
    };

    auto getter2 = [&ip2] {
        ASSERT_FALSE(ip2->is_initialized());
        ASSERT_THROW(ip2->get_nowait<int>(), std::runtime_error);
        // get_new will wait automatically
        auto i1 = ip2->get_new<int>();
        ASSERT_EQ(*i1, 10);
        auto i2 = ip2->get_new<int>();
        ASSERT_EQ(*i2, 100);
    };

    std::thread t1(setter);
    std::thread t2(getter1);
    std::thread t3(getter2);
    t1.join();
    t2.join();
    t3.join();
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
