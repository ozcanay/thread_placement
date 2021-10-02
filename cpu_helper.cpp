#include "cpu_helper.h"
#include "logger_helper.h"
#include "string_helper.h"
#include "constants.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>

long getCoreCount()
{
    return sysconf(_SC_NPROCESSORS_ONLN);
}

std::map<int, int> getMsrFds()
{
    logger->debug(__PRETTY_FUNCTION__);

    std::map<int, int> fd_map;

    char filename[100];

    auto logical_core_count = getCoreCount();
    logger->debug("logical core count: {}", logical_core_count);

    for(auto i = 0; i < logical_core_count; ++i) {
        sprintf(filename, "/dev/cpu/%d/msr",i);

        int fd = open(filename, O_RDWR);

        if(fd >= 0) {
            fd_map.insert({i, fd});
        } else if(fd == -1) {
            logger->error("error on open(): {}", strerror(errno));
        }
    }

    for(const auto& p : fd_map) {
        logger->debug("MSR fd of core {}: {}", p.first, p.second);
    }

    return fd_map;
}

void setAllUncoreRegisters(const std::vector<unsigned int>& vals)
{
    logger->debug(__PRETTY_FUNCTION__);

    logger->debug("Setting all uncore registers with values: ");
    for(auto val : vals) {
        logger->debug("{:x}", val);
    }
    
    long processor_in_socket[NUM_SOCKETS];
    auto logical_core_count = getCoreCount();
    processor_in_socket[0] = 0;
    processor_in_socket[1] = logical_core_count - 1;

    auto fds = getMsrFds();

    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            long core = processor_in_socket[socket];

            for(int i = 0; i < vals.size(); ++i) {
                uint64_t val = vals[i];
                uint64_t offset = CHA_MSR_PMON_CTRL_BASE + (0x10 * cha) + i;

                ssize_t rc64 = pwrite(fds[core], &val, sizeof(val), offset);
                if(rc64 == 8) {
                    logger->debug("Configuring socket {}, CHA {}, by writing 0x{:x} to core {} (fd: {}), offset 0x{:x}.", 
                                                                            socket, cha, val, core, fds[core], offset);
                } else {
                    logger->error("Error writing all data to MSR device on core {}, written {} bytes.", core, rc64);
                }
            }
        }
    }
}

void setAllUncoreMeshTrafficMeasure()
{
    logger->info(__PRETTY_FUNCTION__);
    std::vector<unsigned int> vals{LEFT_READ, RIGHT_READ, UP_READ, DOWN_READ};

    setAllUncoreRegisters(vals);
}

std::vector<std::vector<Tile>> readTopoIntoMatrix(const std::string& topo_file)
{
    std::vector<std::vector<Tile>> tiles(5, std::vector<Tile>(6));

    std::ifstream core_topo;
    core_topo.open("thft_core_topo.txt");
    std::ifstream cha_topo;
    cha_topo.open("thft_cha_topo.txt");

    {
        int row = 0;
        
        std::string line;
        while(std::getline(core_topo, line)) {
            auto cores = split(line);

            int col = 0;
            for(const auto& core : cores) {
                Tile tile;
                tile.row = row;
                tile.col = col;
                tile.core = std::stoi(core);

                tiles[row][col] = tile;

                // std::cout << "core: " << core << std::endl;
                ++col;
            }

            // std::cout << "------" << std::endl;
            
            ++row;
        }
    }

    {
        int row = 0;
        
        std::string line;
        while(std::getline(cha_topo, line)) {
            auto chas = split(line);

            int col = 0;
            for(const auto& cha : chas) {
                tiles[row][col].cha = std::stoi(cha);
                // std::cout << "core: " << core << std::endl;
                ++col;
            }

            // std::cout << "------" << std::endl;
            
            ++row;
        }
    }

    return tiles;
}

/// DO NOT FORGET YX ROUTING! ALWAYS VERTICAL FIRST!! VERTICAL IS LESS COSTLY BTW!
int cycleBetween(const std::vector<std::vector<Tile>>& topo, int core1, int core2, int cha)
{
    std::pair<int, int> core1_loc;
    std::pair<int, int> core2_loc;
    std::pair<int, int> cha_loc;

    for(const auto& row_tiles: topo) {
        for(const auto& tile : row_tiles) {
            if(tile.core == core1) {
                core1_loc = {tile.row, tile.col};    
            } else if(tile.core == core2) {
                core2_loc = {tile.row, tile.col};    
            }

            if(tile.cha == cha) {
                cha_loc = {tile.row, tile.col};
            }           
        }
    }

    const int vertical_hop   = 1;
    const int horizontal_hop = 2;

    int cycle = 0;

    /// between core1 and cha
    cycle += std::abs(core1_loc.first - cha_loc.first) * vertical_hop;
    cycle += std::abs(core1_loc.second - cha_loc.second) * horizontal_hop;
    
    /// between core2 and cha
    cycle += std::abs(cha_loc.first - core2_loc.first) * vertical_hop;
    cycle += std::abs(cha_loc.second - core2_loc.second) * horizontal_hop;

    /// between core2 and core1
    cycle += std::abs(core1_loc.first - core2_loc.first) * vertical_hop;
    cycle += std::abs(core1_loc.second - core2_loc.second) * horizontal_hop;

    return cycle;
}