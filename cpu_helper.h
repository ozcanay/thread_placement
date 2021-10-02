#pragma once

#include <vector>
#include <map>

struct Tile {
    int cha = -1;
    int core = -1;
    int row = -1;
    int col = -1;
};

long getCoreCount();
std::map<int, int> getMsrFds();
void setAllUncoreRegisters(const std::vector<unsigned int>& vals);
void setAllUncoreMeshTrafficMeasure();
std::vector<std::vector<Tile>> readTopoIntoMatrix(const std::string& topo_file);
int cycleBetween(const std::vector<std::vector<Tile>>& topo, int core1, int core2, int cha);



