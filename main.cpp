#include "thread_helper.h"
#include "intel_helper.h"
#include "logger_helper.h"
#include "cpu_helper.h"
#include "constants.h"

#include <x86intrin.h> /// _mm_clflush

#include <algorithm>

spdlog::logger* logger = nullptr;

#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>

using namespace std;

std::mutex g_val_mu;

#include <random>


static inline int generateRandomNum(int lower, int upper)
{
    /// Aditya's way of generating random number.
    unsigned long lo, hi;
    asm volatile( "rdtsc" : "=a" (lo), "=d" (hi) );
    return (lo % 54121) % 100; // mod by a prime.

    // std::random_device dev;
    // std::mt19937 rng(dev());
    // std::uniform_int_distribution<std::mt19937::result_type> dist(lower, upper); // distribution in range [lower, upper]

    // return dist(rng);
}

using namespace std::chrono;

void runMicroBenchmark(std::map<int, int>& msr_fds, const std::vector<unsigned int>& vals, long long* data, long long iteration_count, bool flush);
void runCommMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id);

int main(int argc, char** argv)
{
    setLogger();
    logger->info("================================================ STARTING ================================================");

    auto matrix = readTopoIntoMatrix("thft_core_topo.txt");

    // for(const auto& tiles : matrix) {
    //     for(const auto& tile : tiles) {
    //         logger->debug("row: {}, col: {}, core: {}, cha: {}", tile.row, tile.col, tile.core, tile.cha);
    //     }
    // }

    logger->debug("cycleBetween(matrix, 0, 10, 5): {}", cycleBetween(matrix, 0, 15, 6));

    return 0;

    int main_thread_core_id = 5;
    stick_this_thread_to_core(main_thread_core_id);

    unsigned int COUNTER_CONTROL0 = LLC_LOCAL_LOOKUP;
    unsigned int COUNTER_CONTROL1 = LLC_REMOTE_LOOKUP;
    unsigned int COUNTER_CONTROL2 = LLC_ANY_LOOKUP;
    unsigned int COUNTER_CONTROL3 = LLC_DATA_READ_LOOKUP; 
    unsigned int FILTER0 = FILTER0_ALL_LLC;
    unsigned int FILTER1 = FILTER1_OFF; /// should remain off on my tests.

    logger->info("COUNTER_CONTROL0: {}", descriptions[COUNTER_CONTROL0]);
    logger->info("COUNTER_CONTROL1: {}", descriptions[COUNTER_CONTROL1]);
    logger->info("COUNTER_CONTROL2: {}", descriptions[COUNTER_CONTROL2]);
    logger->info("COUNTER_CONTROL3: {}", descriptions[COUNTER_CONTROL3]);
    logger->info("FILTER0: {}", descriptions[FILTER0]);
    logger->info("FILTER1: {}", descriptions[FILTER1]);

    const long long iteration_count = 1'000'000'000; /// most of my tests had this value 10'000'000'000
    logger->info("ITERATION COUNT: {}", iteration_count);

    int num_elements = CACHE_LINE_SIZE / sizeof(long long);
    logger->debug("num elements: {}", num_elements);
    void *v_data = nullptr;
    /// allocate heap of size equal to just 1 cache line.
    posix_memalign(&v_data, CACHE_LINE_SIZE, num_elements * sizeof(long long));
    long long* data = (long long*)v_data;

    logger->info("allocated virtual data address: {:p}", v_data);
    logger->debug("Initially flushing data from cache...");
    _mm_clflush(&data[0]);
    logger->debug("Initially flushed data from cache.");

    /// setting up msr registers to count llc lookups.
    std::vector<unsigned int> vals{COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3, FILTER0, FILTER1};
    setAllUncoreRegisters(vals);

    vals = {COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3}; /// I WONT READ FILTERS!
    auto msr_fds = getMsrFds();

    logger->flush();
    runMicroBenchmark(msr_fds, vals, data, iteration_count, false);
    logger->flush();
    runMicroBenchmark(msr_fds, vals, data, iteration_count, true);

    logger->info("FIRST PHASE FINISHED.");
    int secondary_thread_core_id = 2;
    runCommMicroBenchmark(data, iteration_count, main_thread_core_id, secondary_thread_core_id);


    logger->info("================================================ ENDING ================================================");
    delete logger;
    logger = nullptr;
    
    return 0;
}

void runMicroBenchmark(std::map<int, int>& msr_fds, const std::vector<unsigned int>& vals, long long* data, long long iteration_count, bool flush)
{
    logger->info("-------------------------------------------------------------- MICROBENCHMARK -------------------------------------------------------------------");
    logger->info("will flush?: {}", flush ? "yes" : "no");
    /// key: cha || value: pair of vectors, each vector representing read counter value, each pair representing before and after values.
    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> read_vals;

    logger->debug("---------------- FIRST READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (0x10*cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].first.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                    // std::cout << "Read " << msr_val << " from socket" << socket << "-CHA" << cha << 
                    // " on core " << core << ", offset 0x" << std::hex << msr_num << std::dec << std::endl;
                }
            }
        }
    }

    if(flush) {
        logger->info("Modifying data and then flushing immediately several times. This should take long.");
        for(long long i = 0; i < iteration_count; ++i) {
            data[0] += 1;
            _mm_clflush(&data[0]);
        }        
    } else {
        logger->info("just iterating through data and incrementing the first item by 1 in each iteration without flushing.");
        for(long long i = 0; i < iteration_count; ++i) {
            data[0] += 1;
        }
    }
    logger->info("DONE.");

    logger->debug("---------------- SECOND READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (0x10*cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].second.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
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
        logger->debug("map entry -> {} : <{} {} {} {}, {} {} {} {}>", read_val.first, 
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
}

void incrementLoop(long long* data, long long iteration_count, int core_id)
{
    stick_this_thread_to_core(core_id);	
    logger->info("Core: {}", sched_getcpu());

    for(int i = 0; i < iteration_count; ++i) {
        std::lock_guard<std::mutex> lg(g_val_mu);
        data[0] += 1;
    }
}

void runCommMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id)
{
	logger->info(">>>>>>>>>>>>>>>>>>>>>");
	logger->info("Microbenchmark started!");

	high_resolution_clock::time_point start;
	high_resolution_clock::time_point end;


    start = high_resolution_clock::now();

    std::thread other_thread(incrementLoop, data, iteration_count, secondary_thread_core_id);
    incrementLoop(data, iteration_count, main_thread_core_id);
    other_thread.join();

    end = high_resolution_clock::now();


	duration<double> time_span = duration_cast<duration<double>>(end - start);
    logger->info("data last value: {}", data[0]);
	logger->info("Start-to-end second: {}", time_span.count());
	logger->info("Microbenchmark ended.");
	logger->info("<<<<<<<<<<<<<<<<<<<<<");
	logger->info("Microbenchmark ended.");
}

