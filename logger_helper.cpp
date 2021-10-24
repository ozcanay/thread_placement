#include "logger_helper.h"

void setLogger(const std::string& log_name)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    console_sink->set_pattern("[thread_placement] [%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_name, false); /// do not truncate file.
    file_sink->set_level(spdlog::level::info);

    /// excluded console_sink below!
    logger = new spdlog::logger("thread_placement", {file_sink});
    logger->flush_on(spdlog::level::info); /// might cause performance bottleneck.
    logger->set_level(spdlog::level::info);
}