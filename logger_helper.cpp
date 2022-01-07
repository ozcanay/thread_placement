#include "logger_helper.h"

#include <iostream>

void setLogger(const std::string& log_name, spdlog::level::level_enum level)
{
    // auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // console_sink->set_level(spdlog::level::info);
    // console_sink->set_pattern("[thread_placement] [%^%l%$] %v");

    // auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_name, false); /// do not truncate file.
    // file_sink->set_level(spdlog::level::info);

    // /// excluded console_sink below!
    // logger = new spdlog::logger("thread_placement", {file_sink});
    // logger->flush_on(spdlog::level::info); /// might cause performance bottleneck.
    // logger->set_level(spdlog::level::info);

    try {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(level);
        console_sink->set_pattern("[multi_sink_example] [%^%l%$] %v");

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_name, false);
        file_sink->set_level(level);

        // auto syslog_logger = std::make_sharedspdlog::syslog_logger_mt("syslog", "pcap-parser", LOG_CONS | LOG_NDELAY, LOG_LOCAL2);
        // syslog_logger->enable_backtrace(32);
        // // spdlog::set_default_logger(syslog_logger);
        // // spdlog::set_level(level);
        // spdlog::flush_on(level);


        logger = std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({/*console_sink, */file_sink}));
        logger->set_level(level);
        logger->flush_on(level);
    } catch (const spdlog::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
}