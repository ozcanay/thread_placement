#pragma once

#include <cstdint> // uint32_t

uint32_t PCI_cfg_index(unsigned int Bus, unsigned int Device, unsigned int Function, unsigned int Offset);
void initializeMMConfigPtr();