#include "system/System.hpp"
#include "utils/Utils.hpp"
#include <chrono>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace KinovaRobustControl::Utils;
namespace KinovaRobustControl
{
namespace System
{
System::System(const std::string &system_name)
    : system_name(system_name), logger(system_name)
{
}

System &System::add_block(BaseBlock::SharedPtr block)
{
    const std::string &block_name = block->get_block_name();
    if (check_block(block_name))
    {
        throw std::runtime_error("Block with name " + block_name +
                                 " already exists!");
    }
    _blocks.insert(std::make_pair(block_name, block));
    return *this;
}

System &System::connect_blocks(const std::string &source_block_name,
                               const std::string &dest_block_name,
                               const std::string &input_name)
{
    assert_block_exist(source_block_name);
    assert_block_exist(dest_block_name);
    return connect_blocks(_blocks.at(source_block_name),
                          _blocks.at(dest_block_name), input_name);
}

System &System::connect_blocks(BaseBlock::SharedPtr src_block,
                               BaseBlock::SharedPtr dst_block,
                               const std::string &input_name)
{
    // helper function for checking
    static const auto block_checker = [this](BaseBlock::SharedPtr blk) {
        // check existence
        if (!check_block(blk->get_block_name()))
            add_block(blk);
        // check if block are the same
        else if (_blocks.at(blk->get_block_name()).get() != blk.get())
        {
            logger.critical("Block with name {} doesn't match existing "
                            "block with the same name!",
                            blk->get_block_name());
            throw std::runtime_error("System Error");
        }
    };

    block_checker(src_block);
    block_checker(dst_block);

    try
    {
        if (input_name.empty())
            dst_block->connect(src_block);
        else
            dst_block->connect(src_block, input_name);
    }
    catch (std::exception &err)
    {
        logger.error("src block {} can't be connected to dst_block {} with "
                     "error message: \n{}",
                     src_block->get_block_name(), dst_block->get_block_name(),
                     err.what());
        throw std::runtime_error("System error");
    }
    catch (...)
    {
        logger.error("src block {} can't be connected to dst_block {} with "
                     "uncatched error.",
                     src_block->get_block_name(), dst_block->get_block_name());
        throw std::runtime_error("System error");
    }
    return *this;
}

BaseBlock::SharedPtr System::get_block(const std::string &block_name)
{
    if (!check_block(block_name))
    {
        throw std::runtime_error("no such block named " + block_name);
    }

    return _blocks.find(block_name)->second;
}

void System::run()
{
    // verbose info
    if (verbose)
    {
        logger.info("Running system {{{}}}", system_name);
        std::stringstream ss;
        for (auto &pair : _blocks)
        {
            ss << pair.second->describe_connection();
        }
        logger.info("\n{}", ss.str());
    }

    std::vector<std::thread> threads;

    // catch error
    try
    {
        // Register signal handler to handle kill signal
        Signal::setupSignalHandlers();
        // register threads
        for (auto &pair : _blocks)
        {
            threads.push_back(run_block(pair.second));
        }

        if (timeout != 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<std::size_t>(timeout * 1000)));
            throw Signal::SignalException(Signal::Signal::Interrupt);
        }

        // wait for finish
        for (auto &t : threads)
        {
            t.join();
        }
    }
    // shutdown system
    catch (Signal::SignalException &e)
    {
        logger.info("SignalException: {}", e.what());
    }
    catch (std::exception &e)
    {
        logger.error("Exception: {}", e.what());
    }
    catch (...)
    {
        logger.error("Unknown error");
    }

    logger.info("Shutting down system...");
    shutdown();

    // wait for finish
    for (auto &t : threads)
    {
        t.join();
    }
}

std::thread System::run_block(BaseBlock::SharedPtr block)
{
    auto wrapper = [this, block]() {
        Logger block_logger("Thread " + system_name + "/" +
                            block->get_block_name());
        try
        {
            block->run();
        }
        catch (Utils::thread_stop &stop)
        {
            block_logger.info("Block <{}> stopped", block->get_block_name());
        }
        catch (std::exception &err)
        {
            block_logger.error("block excpetion: \n{}", err.what());
        }
        catch (...)
        {
            block_logger.error("miss the error? wtf");
        }
    };
    return std::thread(wrapper);
}
} // namespace System
} // namespace KinovaRobustControl
