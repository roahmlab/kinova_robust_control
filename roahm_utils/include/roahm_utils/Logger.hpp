#pragma once
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace Roahm
{
/**
 * @brief wrapper logger for spdlog
 * @details this wrapper will create 2 loggers: stdout and stderr
 *          and assign error() and critical() to stderr
 **/
class Logger
{
  private:
    // loggers
    std::shared_ptr<spdlog::logger> out, err;

    const std::string &name;

  public:
    /**
     * @brief default constructor
     * @param name of the logger
     **/
    Logger(const std::string &name = "",
           spdlog::level::level_enum level = spdlog::level::info)
        : out(spdlog::stdout_color_mt(name)),
          err(spdlog::stderr_color_mt(name + "_err")), name(name)
    {
        out->set_level(level);
        err->set_level(level);
        out->debug("console object initialized");
        err->debug("error object initialized");
    }

    ~Logger()
    {
        spdlog::drop(name);
        spdlog::drop(name + "_err");
    }

    template <typename... Args> void trace(const Args &... msg) const
    {
        this->out->trace(msg...);
    }

    template <typename... Args> void debug(const Args &... msg) const
    {
        this->out->debug(msg...);
    }

    template <typename... Args> void info(const Args &... msg) const
    {
        this->out->info(msg...);
    }

    template <typename... Args> void warn(const Args &... msg) const
    {
        this->out->warn(msg...);
    }

    template <typename... Args> void error(const Args &... msg) const
    {
        this->err->error(msg...);
    }

    template <typename... Args> void critical(const Args &... msg) const
    {
        this->err->critical(msg...);
    }
};
} // namespace Roahm
