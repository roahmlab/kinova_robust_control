#pragma once
#include "system/BaseBlock.hpp"
#include <utils/Logger.hpp>
#include <utils/Utils.hpp>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace KinovaRobustControl
{
namespace System
{
/**
 * @brief wrapper class for the whole system
 **/
class System : private Utils::noncopyable
{
  private:
    std::unordered_map<std::string, BaseBlock::SharedPtr> _blocks;
    const std::string system_name;
    Logger logger;
    bool verbose{false};
    double timeout{0};

  public:
    /**
     * @brief ctor
     **/
    System(const std::string &system_name);

    /**
     * @brief add a new block to system
     * @return ref of this
     **/
    System &add_block(BaseBlock::SharedPtr block);

    /**
     * @brief connect blocks
     * @param source_block_name: name of the source block
     * @param dest_block_name: name of the destination block
     * @param input_name: name of the input port, optional if source_block type
     *        is unique in destination input types
     * @return ref of this
     **/
    System &connect_blocks(const std::string &source_block_name,
                           const std::string &dest_block_name,
                           const std::string &input_name = "");

    /**
     * @brief connect block, taking block sharedptr as input
     * @warning unregistered block will be added into system
     * @param src_block: source block
     * @param dst_block: destination block
     * @param input_name: optional input_name if src type is unique in dst
     *        inputs
     * @return ref of this
     **/
    System &connect_blocks(BaseBlock::SharedPtr src_block,
                           BaseBlock::SharedPtr dst_block,
                           const std::string &input_name = "");


    /**
     * @brief get a sharedptr of the block
     * @param block_name: name of the block
     **/
    BaseBlock::SharedPtr get_block(const std::string &block_name);

    /**
     * @brief check whether a block exists
     * @param block_name: name of the block
     **/
    inline bool check_block(const std::string &block_name)
    {
        return (_blocks.find(block_name) != _blocks.end());
    }

    /**
     * @brief enable verbose output
     **/
    inline System &set_verbose()
    {
        verbose = true;
        return *this;
    }

    /**
     * @brief set timeout for system
     * @details 0 for no timeout and need manual stop
     **/
    inline System &set_timeout(double timeout)
    {
        this->timeout = timeout;
        return *this;
    }

    /**
     * @brief run the whole system
     **/
    void run();

  private:
    /**
     * @brief check the block existence and throw error if not exist
     * @param block_name: name of the block
     **/
    inline void assert_block_exist(const std::string &block_name)
    {
        if (!check_block(block_name))
        {
            logger.error("Block {} doesn't exist!", block_name);
            throw std::out_of_range("Block " + block_name + " doesn't exist!");
        }
    }

    /**
     * @brief construct and return a thread running a block function
     **/
    std::thread run_block(BaseBlock::SharedPtr block);

    /**
     * @brief shutdown all the running thread
     **/
    inline void shutdown()
    {
        for (auto b : _blocks)
            b.second->set_stop();
    }
};
} // namespace System
} // namespace KinovaRobustControl
