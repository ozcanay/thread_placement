#pragma once

#include <vector>
#include <map>

long getCoreCount();
std::map<int, int> getMsrFds();
void setAllUncoreRegisters(const std::vector<unsigned int>& vals);
void setAllUncoreMeshTrafficMeasure();