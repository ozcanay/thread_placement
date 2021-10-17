#include "logger_helper.h"

void setLogger()
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    console_sink->set_pattern("[thread_placement] [%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/traffic_log.txt", false); /// do not truncate file.
    file_sink->set_level(spdlog::level::info);

    /// excluded console_sink below!
    logger = new spdlog::logger("thread_placement", {file_sink});
    logger->set_level(spdlog::level::info);
}