#pragma once
#include "system/Port/OutputPort.hpp"
#include "utils/Utils.hpp"
#include <memory>
#include <stdexcept>

namespace KinovaRobustControl
{
namespace System
{
/// forward declaration
template <typename T> class InputPort;

/**
 * @brief Abstract Input Port Interface
 **/
class AbstractInputPort
    : public std::enable_shared_from_this<AbstractInputPort>,
      private Utils::noncopyable
{
  public:
    using SharedPtr = std::shared_ptr<AbstractInputPort>;
    using WeakPtr = std::weak_ptr<AbstractInputPort>;

  public:
    /**
     * @brief ctor
     * @param derived_info: type info for derived class
     * @param data_info: type info for data
     * @param src: optional output port
     **/
    AbstractInputPort(
        const std::type_info &derived_info, const std::type_info &data_info,
        AbstractOutputPort::WeakPtr src = AbstractOutputPort::WeakPtr())
        : data_version(0), derived_type(derived_info), data_type(data_info),
          source(src)
    {
        // perform type checking if %src provided
        if (is_connected())
            check_source_type();
    }

    /**
     * @brief connect inputport to output source
     * @param src: output port to connect
     **/
    void connect(AbstractOutputPort::WeakPtr src)
    {
        source = src;
        check_source_type();
    }

    /**
     * @brief get the latest data without blocking
     * @warning User is responsible to make sure that
     *          the output port is already initialized
     * @return const reference to data stored
     **/
    template <typename T> std::shared_ptr<const T> get_nowait()
    {
        auto shared_src{get_shared_source()};
        shared_src->get_nowait(derived<T>().get_mutable(), data_version);
        // check data version
        if (data_version == 0)
            throw std::runtime_error("Output Port not initialized yet!");

        // return reference
        return derived<T>().get();
    }

    /**
     * @brief get an updated value, block execution to wait for update
     **/
    template <typename T> std::shared_ptr<const T> get_new()
    {
        auto shared_src{get_shared_source()};
        shared_src->get_new(derived<T>().get_mutable(), data_version);

        return derived<T>().get();
    }

    /**
     * @brief get the current version of the data
     **/
    inline std::size_t get_data_version()
    {
        return data_version;
    }

    /**
     * @brief check connection
     * @return whether port is connected
     **/
    inline bool is_connected()
    {
        return !source.expired();
    }

    /**
     * @brief check if output port is initialized
     **/
    bool is_initialized()
    {
        auto shared_src{get_shared_source()};
        bool latest_version = shared_src->query_data_version();
        return (latest_version != 0);
    }

    /**
     * @brief wait for output port initialization
     **/
    void wait_init()
    {
        auto shared_src{get_shared_source()};
        shared_src->wait_init();
    }

    /**
     * @brief determine if given data type match port data type
     * @param dtype: given data type
     **/
    inline bool is_data_compatible(const std::type_info &dtype)
    {
        return dtype == data_type;
    }

  private:
    /**
     * @brief cast self to derived class so that derived class method can be
     *        used
     * @return reference to *this in derived type
     **/
    template <typename T> InputPort<T> &derived()
    {
        // check type
        if (typeid(InputPort<T>) != this->derived_type)
            throw std::runtime_error("requested type " +
                                     std::string(typeid(InputPort<T>).name()) +
                                     " doesn't match storage type " +
                                     std::string(derived_type.name()));
        return static_cast<InputPort<T> &>(*this);
    }

    /**
     * @brief get a shared pointer to output source, throw error if not
     *        connected
     * @return shared pointer to output port
     **/
    AbstractOutputPort::SharedPtr get_shared_source()
    {
        if (AbstractOutputPort::SharedPtr shared_src{source.lock()})
        {
            return shared_src;
        }

        // source not lockable, throw error
        throw std::runtime_error("Input Port not connected!");
    }

    /**
     * @brief check whether the output source type match
     **/
    void check_source_type()
    {
        auto shared_src{get_shared_source()};
        const std::type_info &output_type{shared_src->query_data_info()};
        if (output_type != data_type)
        {
            throw std::runtime_error("Output type " +
                                     std::string(output_type.name()) +
                                     " doesn't match Input Port type " +
                                     std::string(data_type.name()));
        }
    }

  private:
    std::size_t data_version;
    const std::type_info &derived_type;
    const std::type_info &data_type;
    std::weak_ptr<AbstractOutputPort> source;
};

template <typename T> class InputPort final : public AbstractInputPort
{
  private:
    /// pointer of data, don't have write permission
    std::shared_ptr<const T> _data;

  public:
    static AbstractInputPort::SharedPtr make_shared()
    {
        std::unique_ptr<AbstractInputPort> up{nullptr};
        up.reset(new InputPort<T>());
        return up;
    }

    /// dad is my friend
    friend class AbstractInputPort;

  private:
    /**
     * @brief default ctor
     **/
    InputPort()
        : AbstractInputPort(typeid(InputPort<T>), typeid(T)), _data(nullptr)
    {
    }

    /**
     * @brief get another shared_ptr to data
     **/
    inline std::shared_ptr<const T> get() const
    {
        // explicit convert
        return _data;
    }

    /**
     * @brief get a mutable reference to _data
     **/
    inline std::shared_ptr<const T> &get_mutable()
    {
        return _data;
    }
};
} // namespace System
} // namespace KinovaRobustControl
