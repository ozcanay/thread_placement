#pragma once

#include <string>

void readImcCounters();
void configureImcCounters(const std::string& config_file);
void runImcBenchmark(bool flush = true);