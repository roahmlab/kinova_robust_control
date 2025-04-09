#include "utils/Logger.hpp"
#include <spdlog/common.h>

int main()
{
    using namespace KinovaRobustControl;
    Logger logger("example", spdlog::level::trace);
    logger.trace("trace msg");
    logger.debug("debug msg");
    logger.info("info msg: {}", 20);
    logger.warn("msg: {}", "warning");
    logger.error("{1} {0}", "error", "msg:");
    logger.critical("critical msg");
}
