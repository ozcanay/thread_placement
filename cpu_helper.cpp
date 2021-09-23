#include "cpu_helper.h"
#include "logger_helper.h"
#include "constants.h"

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>

long getCoreCount()
{
    return sysconf(_SC_NPROCESSORS_ONLN);
}

std::map<int, int> getMsrFds()
{
    logger->info(__PRETTY_FUNCTION__);

    std::map<int, int> fd_map;

    char filename[100];

    auto logical_core_count = getCoreCount();
    logger->info("logical core count: {}", logical_core_count);

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
        logger->info("MSR fd of core {}: {}", p.first, p.second);
    }

    return fd_map;
}

void setAllUncoreRegisters(const std::vector<unsigned int>& vals)
{
    logger->info(__PRETTY_FUNCTION__);

    logger->info("Setting all uncore registers with values: ");
    for(auto val : vals) {
        logger->info("{:x}", val);
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
                    logger->info("Configuring socket {}, CHA {}, by writing 0x{:x} to core {} (fd: {}), offset 0x{:x}.", 
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