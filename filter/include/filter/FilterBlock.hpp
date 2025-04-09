#pragma once
#include "system/BaseBlock.hpp"
#include <system/Port/InputPort.hpp>
#include <system/Port/OutputPort.hpp>
namespace KinovaRobustControl
{
namespace Filter
{
using KinovaRobustControl::System::BaseBlock;
template <typename T, typename FilterT>
class FilterBlock final : public BaseBlock
{
  public:
    /**
     * @brief maker
     * @param name: name of the block
     * @param FilterT: filter to use
     **/
    static BaseBlock::SharedPtr make_shared(const std::string &name,
                                            FilterT &&filter)
    {
        return BaseBlock::SharedPtr(
            new FilterBlock<T, FilterT>(name, std::move(filter)));
    }

    virtual void run() override
    {
        // filter received data and output
        while (true)
        {
            auto ptr = input->get_new<T>();
            default_output->set(filter.filter(*ptr));
        }
    }

  private:
    FilterBlock(const std::string &name, FilterT &&filter)
        : BaseBlock(name, System::OutputPort<T>::make_shared()),
          input(register_input<T>("input")), filter(std::move(filter))
    {
    }

    System::AbstractInputPort::SharedPtr input;
    FilterT filter;
};
} // namespace Filter
} // namespace KinovaRobustControl
