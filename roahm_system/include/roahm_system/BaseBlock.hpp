/**
 * @brief basic block type
 * @file
 **/
#pragma once
#include "roahm_system/Port/InputPort.hpp"
#include "roahm_system/Port/OutputPort.hpp"
#include "roahm_system/Ports.hpp"
#include "roahm_utils/Logger.hpp"
#include "roahm_utils/Utils.hpp"
#include <atomic>
#include <memory>
#include <stdexcept>
#include <typeinfo>
#include <unordered_map>
#include <vector>

namespace Roahm
{
namespace System
{
/**
 * @brief basic block type
 **/
class BaseBlock : public std::enable_shared_from_this<BaseBlock>,
                  private Utils::noncopyable
{
  public:
    using SharedPtr = std::shared_ptr<BaseBlock>;
    using WeakPtr = std::weak_ptr<BaseBlock>;

  public:
    /**
     * @brief start execution of block
     * @warning user should add check_stop() when doing loop
     **/
    virtual void run() = 0;

    /**
     * @brief destructor
     **/
    ~BaseBlock()
    {
        // logger.info("Destroying Block <{}>", block_name);
    }

    /**
     * @brief automatically search and connect port using port data
     * @warning can only be used when only one inputport match output type
     * @param src_block: source block to connect
     **/
    void connect(BaseBlock::SharedPtr src_block);

    /**
     * @brief connect output port from another block to block with given name
     * @param src_block: %SharedPtr of another block
     * @param port_name: input port name to connect to
     **/
    void connect(BaseBlock::SharedPtr src_block, const std::string &port_name);

    /**
     * @brief describe connection info
     **/
    std::string describe_connection() const;

    /**
     * @brief check if block has input port with given name
     * @param port_name: name of input port
     **/
    inline bool has_input(const std::string &port_name) const
    {
        return (_inputs.find(port_name) != _inputs.end());
    }

    /**
     * @brief get block_name
     **/
    inline const std::string &get_block_name()
    {
        return block_name;
    }

    /**
     * @brief notify block of stopping
     **/
    inline void set_stop()
    {
        _stop = true;
        // also signal outputport and all inputports connected to it
        for (const auto &oport : _outputs)
        {
            oport.second->signal_stop();
        }
    }

    /**
     * @brief check if block should stop
     **/
    inline void check_stop()
    {
        if (_stop)
        {
            throw Utils::thread_stop();
        }
    }

  protected:
    /**
     * @brief default ctor with template argument indicating data type of output
     * @param name: name of the block
     * @param output: shared pointer to output port
     **/
    explicit BaseBlock(const std::string &block_name,
                       AbstractOutputPort::SharedPtr output,
                       const std::string &output_name = "default");

    /**
     * @brief get a weak reference from the output port
     * @param port_name: name of the output port to be used
     *        default empty, can be used if block has only one output port
     * @details declared as protected to prevent overuse
     **/
    inline AbstractOutputPort::WeakPtr weak_output_port(
        const std::string &port_name = "") const
    {
        return _search_output_port(port_name)->weak_from_this();
    }

    /**
     * @brief get a shared reference from the output port
     * @param port_name: name of the output port to be used
     *        default empty, can be used if block has only one output port
     * @details declared as protected to prevent overuse
     **/
    inline AbstractOutputPort::SharedPtr shared_output_port(
        const std::string &port_name = "") const
    {
        return _search_output_port(port_name);
    }

    /**
     * @brief register new input to name
     * @param IN_TYPE: input data type for port
     * @param name: name of input port
     * @return a shared pointer to the InputPort
     **/
    template <typename IN_TYPE>
    AbstractInputPort::SharedPtr register_input(const std::string &port_name)
    {
        // already registered
        if (has_input(port_name))
            throw std::out_of_range("InputPort " + port_name +
                                    " already registered for BaseBlock " +
                                    block_name);

        // insert new input port
        AbstractInputPort::SharedPtr iport(InputPort<IN_TYPE>::make_shared());
        _inputs.insert(std::make_pair(port_name, iport));
        return iport;
    }

  protected:
    /// logger
    ::Roahm::Logger logger;

    AbstractOutputPort::SharedPtr default_output;

  private:
    /**
     * @brief search for outputport
     * @param port_name: name of the port. If empty then default to the first
     *        one stored
     **/
    AbstractOutputPort::SharedPtr _search_output_port(
        const std::string &port_name) const;

    /// ports map
    /// NOTE: Derived class should store another sharedPtr
    ///       instead of searching in HashMap everytime
    std::unordered_map<std::string, AbstractOutputPort::SharedPtr> _outputs;

    std::unordered_map<std::string, AbstractInputPort::SharedPtr> _inputs;

    /// map storing which block the input is linked to
    std::unordered_map<std::string, BaseBlock::WeakPtr> _input_links;

    /// name of block
    const std::string block_name;

    std::atomic_bool _stop{false};
};
} // namespace System
} // namespace Roahm
