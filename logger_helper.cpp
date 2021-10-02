#include "logger_helper.h"

void setLogger()
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[thread_placement] [%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/log.txt", false); /// do not truncate file.
    file_sink->set_level(spdlog::level::debug);

    logger = new spdlog::logger("multi_sink", {console_sink, file_sink});
    logger->set_level(spdlog::level::debug);
}