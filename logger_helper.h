#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/syslog_sink.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>


extern std::shared_ptr<spdlog::logger> logger;

void setLogger(const std::string& log_name, spdlog::level::level_enum level);