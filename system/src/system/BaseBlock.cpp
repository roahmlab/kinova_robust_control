#include "system/BaseBlock.hpp"
#include <sstream>

namespace KinovaRobustControl
{
namespace System
{
void BaseBlock::connect(BaseBlock::SharedPtr src_block)
{
    const std::type_info &output_type{
        src_block->shared_output_port()->query_data_info()};

    std::string port_name{""};

    // find port with desired type
    bool found{false};
    for (const auto &iport : _inputs)
    {
        if (iport.second->is_data_compatible(output_type))
        {
            // duplicate, auto resolve failed
            if (found)
            {
                throw std::runtime_error(
                    "Both port " + port_name + " and port " + iport.first +
                    " has data type " + std::string(output_type.name()));
            }
            found = true;
            port_name = iport.first;
        }
    }

    // no available port found
    if (!found)
    {
        throw std::runtime_error("No port with type " +
                                 std::string(output_type.name()) + " found!");
    }

    // connect with selected name
    connect(src_block, port_name);
}

void BaseBlock::connect(BaseBlock::SharedPtr src_block,
                        const std::string &port_name)
{
    // verify existence
    if (!has_input(port_name))
    {
        throw std::range_error("InputPort " + port_name +
                               " doesn't exist for BaseBlock " + block_name);
    }

    // check connection
    auto iport = _inputs.at(port_name);
    if (iport->is_connected())
    {
        logger.warn("Reconnecting input port {} to BaseBlock {}", port_name,
                    src_block->get_block_name());
    }
    iport->connect(src_block->weak_output_port());

    _input_links[port_name] = src_block->weak_from_this();
}

std::string BaseBlock::describe_connection() const
{
    std::stringstream info;
    info << "Block <" << block_name << ">: \n";
    for (auto &pair : _inputs)
    {
        if (_input_links.find(pair.first) != _input_links.end())
        {
            info << "[" << pair.first << "] -> ";
            if (auto src_blk{_input_links.at(pair.first).lock()})
            {
                info << "<" << src_blk->get_block_name() << ">\n";
            }
            else
            {
                info << "Expired Block\n";
            }
        }
        else
        {
            info << "[" << pair.first << "] -> Nothing\n";
        }
    }
    return info.str();
}

BaseBlock::BaseBlock(const std::string &block_name,
                     AbstractOutputPort::SharedPtr output,
                     const std::string &output_name)
    : logger(block_name), default_output(output), _outputs(),
      block_name(block_name)
{
    _outputs.insert(std::make_pair(output_name, output));
}

AbstractOutputPort::SharedPtr BaseBlock::_search_output_port(
    const std::string &port_name) const
{
    // default argument, return the only one
    if (port_name.empty() && _outputs.size() == 1)
        return default_output;

    // search for port
    auto port_found = _outputs.find(port_name);
    if (port_found == _outputs.end())
    {
        logger.critical("Block {} has no output port with name <{}>",
                        block_name, port_name);
        throw std::runtime_error("BlockException");
    }
    return port_found->second;
}
} // namespace System
} // namespace KinovaRobustControl
