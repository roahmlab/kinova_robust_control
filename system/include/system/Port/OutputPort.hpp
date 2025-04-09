/**
 * @brief output data port object
 **/
#pragma once
#include "utils/Utils.hpp"
#include <atomic>
#include <condition_variable>
#include <exception>
#include <memory>
#include <shared_mutex>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>

namespace KinovaRobustControl
{
namespace System
{
/// forward declaration
template <typename T> class OutputPort;

/**
 * @brief abstract output port base class, shouldn't be directly used
 **/
class AbstractOutputPort
    : public std::enable_shared_from_this<AbstractOutputPort>,
      private Utils::noncopyable
{
  public:
    using SharedPtr = std::shared_ptr<AbstractOutputPort>;
    using WeakPtr = std::weak_ptr<AbstractOutputPort>;

  public:
    /**
     * @brief constructor
     * @param derived: derived type info for checking
     **/
    AbstractOutputPort(const std::type_info &derived_info,
                       const std::type_info &data_info)
        : data_version(0), derived_type(derived_info), data_type(data_info)
    {
    }

    /**
     * @brief set the value
     * @details calling this function will increment data_version
     * @param val: value to set
     **/
    template <typename T> void set(const T &val)
    {
        ulock lk(output_mtx);
        // call derived class set method
        derived<T>().derived_set(val);
        data_version++;
        // notify other thread that resources are ready
        lk.unlock();
        output_cv.notify_all();
    }

    /**
     * @brief signal the input to stop
     **/
    void signal_stop()
    {
        _stop = true;
        // notify waiting thread
        output_cv.notify_all();
    }

    /** @brief check if is stopping */
    inline void check_stop()
    {
        if (_stop)
            throw Utils::thread_stop();
    }

    /**
     * @brief try to get the data once getting the lock
     * @details if query version is up to date, just return without update
     * @param val: value to write into
     * @param version: version to check and update
     **/
    template <typename T>
    inline void get_nowait(std::shared_ptr<const T> &val, std::size_t &version)
    {
        get(val, version, false);
    }

    /**
     * @brief guaranteed to get a new version of value
     * @details if query version is up to date, block and wait for new one
     *          else just return the current one
     * @param val: value to write into
     * @param version: version to check and update
     **/
    template <typename T>
    inline void get_new(std::shared_ptr<const T> &val, std::size_t &version)
    {
        get(val, version, true);
    }

    /**
     * @brief wait for initialization
     * @warning inputport may want to check for this before entering the loop
     **/
    void wait_init()
    {
        slock lk(output_mtx);

        // wait until data is updated
        output_cv.wait(lk, [this] { return data_version > 0; });
    }

    /**
     * @brief get the current version of data
     * @return current data version
     **/
    std::size_t query_data_version()
    {
        slock lk(output_mtx);
        return data_version;
    };

    /**
     * @brief get data type information
     **/
    const std::type_info &query_data_info()
    {
        return data_type;
    }

    /**
     * @brief get shared_ptr to the instance
     **/
    inline std::shared_ptr<AbstractOutputPort> shared()
    {
        return shared_from_this();
    }

  private:
    /// lock raii alias
    using ulock = std::unique_lock<std::shared_mutex>;
    using slock = std::shared_lock<std::shared_mutex>;

    /**
     * @brief cast self to derived class so that derived class method can be
     *        used
     * @return reference to *this in derived type
     **/
    template <typename T> OutputPort<T> &derived()
    {
        // check type
        if (typeid(OutputPort<T>) != this->derived_type)
            throw std::runtime_error("requested type " +
                                     std::string(typeid(OutputPort<T>).name()) +
                                     " doesn't match storage type " +
                                     std::string(derived_type.name()));
        return static_cast<OutputPort<T> &>(*this);
    }

    /**
     * @brief helper function of get
     * @param val: value to write into
     * @param version: version to check and update
     * @param wait: whether to wait for new update
     **/
    template <typename T>
    void get(std::shared_ptr<const T> &val, std::size_t &version, bool wait)
    {
        slock lk(output_mtx);

        // check if need to stop
        check_stop();

        // version up to date
        if (version == data_version)
        {
            // just return if nowait
            if (wait)
            {
                output_cv.wait(lk, [this, version] {
                    check_stop();
                    return (version < data_version);
                });
            }
            else
            {
                return;
            }
        }

        // update value and version
        val = derived<T>().get();
        version = data_version;
    }

  private:
    /// mutex and cv for data stored
    std::shared_mutex output_mtx;
    std::condition_variable_any output_cv;

    /// version of current data
    std::size_t data_version;

    /// typeid for verification
    const std::type_info &derived_type;
    const std::type_info &data_type;

    std::atomic_bool _stop{false};
};

/**
 * @brief actual output port to be used
 * @NOTE T should be default constructible and copy constructible
 * TODO: how to check default constructible
 **/
template <typename T> class OutputPort final : public AbstractOutputPort
{
  private:
    /// pointer of data to store
    std::shared_ptr<T> _data;

  public:
    /**
     * @brief maker for unified storage
     **/
    static std::shared_ptr<AbstractOutputPort> make_shared()
    {
        std::unique_ptr<AbstractOutputPort> up{nullptr};
        up.reset(new OutputPort<T>());
        return up;
    }

    /// dad is my friend
    friend AbstractOutputPort;

  private:
    /**
     * @brief default constructor
     * @details can only be called by maker
     **/
    OutputPort()
        : AbstractOutputPort(typeid(OutputPort<T>), typeid(T)), _data(nullptr)
    {
    }

    /**
     * @brief write to data the given value
     * @warning to be called by base class
     * @param val: value to be wrote
     **/
    void derived_set(const T &val)
    {
        // copy constructor
        _data = std::make_shared<T>(val);
    }

    /**
     * @brief get value
     * @warning to be called by base class
     * @return shared pointer with const tag
     **/
    std::shared_ptr<const T> get()
    {
        return std::shared_ptr<const T>{_data};
    }
};
} // namespace System
} // namespace KinovaRobustControl
