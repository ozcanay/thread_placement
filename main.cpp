#include "thread_helper.h"
#include "intel_helper.h"
#include "logger_helper.h"
#include "cpu_helper.h"
#include "constants.h"

#include <x86intrin.h> /// _mm_clflush

#include <algorithm>

spdlog::logger* logger = nullptr;

#include <unistd.h>
#include <iostream>
#include <chrono>

using namespace std::chrono;

int main(int argc, char** argv)
{
    setLogger();

    /// I PROVED HERE THAT CLFLUSH DRAMATICALLY IMPACTS PERFORMANCE.
    // stick_this_thread_to_core(5);

    // long long a = 3;

    // logger->info("Entering for loop WITH CLFLUSH...");

    // high_resolution_clock::time_point start = high_resolution_clock::now();
    // for(long long i = 0; i < 1'000'000'000; ++i) {
    //     _mm_clflush(&a);
    //     ++a;
    // }
    // high_resolution_clock::time_point end = high_resolution_clock::now();
    // duration<double> time_span = duration_cast<duration<double>>(end - start);
	// logger->info("Start-to-end second: {}", time_span.count());


    // logger->info("a: {}", a);

    // return 0;

    unsigned int COUNTER_CONTROL0 = LLC_ANY_LOOKUP;
    unsigned int COUNTER_CONTROL1 = LLC_ANY_LOOKUP;
    unsigned int COUNTER_CONTROL2 = LLC_ANY_LOOKUP;
    unsigned int COUNTER_CONTROL3 = LLC_ANY_LOOKUP; 
    unsigned int FILTER0 = FILTER0_ALL_LLC;
    unsigned int FILTER1 = FILTER1_OFF; /// should remain off on my tests.

    int num_elements = CACHE_LINE_SIZE / sizeof(long long);
    logger->info("num elements: {}", num_elements);
    void *v_data = nullptr;
    posix_memalign(&v_data, CACHE_LINE_SIZE, num_elements * sizeof(long long));
    long long* data = (long long*)v_data;

    logger->info("Flushing data from cache...");
    _mm_clflush(&data[0]);
    logger->info("Flushed data from cache.");

    /// setting up msr registers to count llc lookups.
    std::vector<unsigned int> vals{COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3, FILTER0, FILTER1};
    setAllUncoreRegisters(vals);

    vals = {COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3}; /// I WONT READ FILTERS!
    auto msr_fds = getMsrFds();

    /// key: cha || value: pair of vectors, each vector representing read counter value, each pair representing before and after values.
    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> read_vals;

    logger->info("---------------- FIRST READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (0x10*cha) + i;
                logger->info("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].first.push_back(msr_val);
                    logger->info("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                    // std::cout << "Read " << msr_val << " from socket" << socket << "-CHA" << cha << 
                    // " on core " << core << ", offset 0x" << std::hex << msr_num << std::dec << std::endl;
                }
            }
        }
    }

    logger->info("Modifying data and then flushing immediately several times. This should take long.");
    for(long long i = 0; i < 100'000'000; ++i) {
        data[0] += 1;
        _mm_clflush(&data[0]);
    }
    logger->info("DONE.");

    logger->info("---------------- SECOND READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (0x10*cha) + i;
                logger->info("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].second.push_back(msr_val);
                    logger->info("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                    // std::cout << "Read " << msr_val << " from socket" << socket << "-CHA" << cha << 
                    // " on core " << core << ", offset 0x" << std::hex << msr_num << std::dec << std::endl;
                }
            }
        }
    }

    logger->info("---------------- ANALYZING ----------------");
    long long sum_diff0 = 0;
    long long sum_diff1 = 0;
    long long sum_diff2 = 0;
    long long sum_diff3 = 0;

    for(const auto& read_val : read_vals) {
        logger->info("map entry -> {} : <{} {} {} {}, {} {} {} {}>", read_val.first, 
        read_val.second.first[0], read_val.second.first[1], read_val.second.first[2], read_val.second.first[3], 
        read_val.second.second[0], read_val.second.second[1], read_val.second.second[2], read_val.second.second[3]);

        logger->info("CHA: {}, read diff: <{} {} {} {}>", read_val.first, 
                                                    read_val.second.second[0] - read_val.second.first[0],
                                                    read_val.second.second[1] - read_val.second.first[1],
                                                    read_val.second.second[2] - read_val.second.first[2],
                                                    read_val.second.second[3] - read_val.second.first[3]);

        sum_diff0 += (read_val.second.second[0] - read_val.second.first[0]);
        sum_diff1 += (read_val.second.second[1] - read_val.second.first[1]);
        sum_diff2 += (read_val.second.second[2] - read_val.second.first[2]);
        sum_diff3 += (read_val.second.second[3] - read_val.second.first[3]);
    }

    logger->info("Summarizing in terms of percentage:");
    for(const auto& read_val : read_vals) {
        logger->info("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", read_val.first, 
        ((read_val.second.second[0] - read_val.second.first[0]) / (double)sum_diff0) * 100,
        ((read_val.second.second[1] - read_val.second.first[1]) / (double)sum_diff1) * 100,
        ((read_val.second.second[2] - read_val.second.first[2]) / (double)sum_diff2) * 100,
        ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100);
    }

    delete logger;
}