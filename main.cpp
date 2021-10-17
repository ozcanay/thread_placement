#include "thread_helper.h"
#include "intel_helper.h"
#include "logger_helper.h"
#include "cpu_helper.h"
#include "constants.h"

#include <x86intrin.h> /// _mm_clflush

#include <algorithm>

spdlog::logger* logger = nullptr;

#include <set>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <climits>

using namespace std;

std::mutex g_val_mu;
std::condition_variable cv;
volatile bool write_{false};
volatile bool read_{false};

std::vector<int> thread_modify_orders_; /// 2 threads should modify variable in turns. I will make sure they do it in turn using this vector. expecting for ids 0 1 0 1 0 1 0 1 --> no consecutive thread ids inside.

#include <random>

static __inline__ unsigned long long rdtsc(void)
{
    unsigned hi, lo;
    __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
    return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}


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

int runMicroBenchmark(std::map<int, int>& msr_fds, const std::vector<unsigned int>& vals, long long* data, long long iteration_count, bool flush);
double runCommMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const std::vector<unsigned int>& vals, std::map<int, int>& msr_fds);
void incrementLoop(long long* data, long long iteration_count, int core_id);


int main(int argc, char** argv)
{
    if(argc != 3) {
        std::cout << "./placer <CORE_NO> <CORE_NO>" << std::endl;
        return 0;
    }

    thread_modify_orders_.reserve(40000000);

    std::signal(SIGHUP, SIG_IGN);

    setLogger();
    logger->info("================================================ STARTING ================================================");
    logger->debug("WILL USE RDTSC");

    auto matrix = readTopoIntoMatrix("thft_core_topo.txt");
    
    int main_thread_core_id = std::stoi(argv[1]);
    int secondary_thread_core_id = std::stoi(argv[2]);;

    stick_this_thread_to_core(main_thread_core_id);


    unsigned int COUNTER_CONTROL0 = LEFT_READ;
    unsigned int COUNTER_CONTROL1 = UP_READ;
    unsigned int COUNTER_CONTROL2 = DOWN_READ;
    unsigned int COUNTER_CONTROL3 = LLC_DATA_READ_LOOKUP; /// DO NOT CHANGE THIS, I USE THIS TO FIND ASSIGNED CHA OF A MEMORY ADDRESS.
    unsigned int FILTER0 = FILTER0_ALL_LLC;
    unsigned int FILTER1 = FILTER1_OFF; /// should remain off on my tests.

    logger->debug("COUNTER_CONTROL0: {}", descriptions[COUNTER_CONTROL0]);
    logger->debug("COUNTER_CONTROL1: {}", descriptions[COUNTER_CONTROL1]);
    logger->debug("COUNTER_CONTROL2: {}", descriptions[COUNTER_CONTROL2]);
    logger->debug("COUNTER_CONTROL3: {}", descriptions[COUNTER_CONTROL3]);
    logger->debug("FILTER0: {}", descriptions[FILTER0]);
    logger->debug("FILTER1: {}", descriptions[FILTER1]);

    // const long long iteration_count = 1'000'000'000;
    const long long iteration_count = 50'000'000;
    // const long long iteration_count = 100;
    // const long long iteration_count = 10;

    logger->info("ITERATION COUNT: {}", iteration_count);

    int num_elements = CACHE_LINE_SIZE / sizeof(long long);
    logger->debug("num elements: {}", num_elements);
    void *v_data = nullptr;
    /// allocate heap of size equal to just 1 cache line.
    posix_memalign(&v_data, CACHE_LINE_SIZE, num_elements * sizeof(long long));
    long long* data = (long long*)v_data;

    logger->info("allocated virtual data address: {:p}", v_data);
    logger->debug("Initially flushing data from cache...");
    _mm_mfence();
    _mm_clflush(&data[0]);
    _mm_mfence();
    logger->debug("Initially flushed data from cache.");

    /// setting up msr registers to count llc lookups.
    std::vector<unsigned int> vals{COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3, FILTER0, FILTER1};
    setAllUncoreRegisters(vals);

    vals = {COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3}; /// I WONT READ FILTERS!
    auto msr_fds = getMsrFds();

    // logger->flush();
    // runMicroBenchmark(msr_fds, vals, data, iteration_count, false);
    logger->flush();
    auto assigned_cha = runMicroBenchmark(msr_fds, vals, data, iteration_count, true);

    logger->info("COUNTER_CONTROL0: {}", descriptions[COUNTER_CONTROL0]);
    logger->info("COUNTER_CONTROL1: {}", descriptions[COUNTER_CONTROL1]);
    logger->info("COUNTER_CONTROL2: {}", descriptions[COUNTER_CONTROL2]);
    logger->info("COUNTER_CONTROL3: {}", descriptions[COUNTER_CONTROL3]);
    logger->info("FILTER0: {}", descriptions[FILTER0]);
    logger->info("FILTER1: {}", descriptions[FILTER1]);

    logger->info("core: {}, core: {}", main_thread_core_id, secondary_thread_core_id);
    logger->info("cha: {}", assigned_cha);
    logger->info("iteration_count: {}", iteration_count);

    int total_hop = cycleBetween(matrix, main_thread_core_id, secondary_thread_core_id, assigned_cha);
    logger->info("total hop: {}", total_hop);

    logger->info("CHA assign phase finished.");
    logger->flush();

    COUNTER_CONTROL0 = LEFT_AK_READ;
    COUNTER_CONTROL1 = RIGHT_AK_READ;
    COUNTER_CONTROL2 = UP_AK_READ;
    COUNTER_CONTROL3 = DOWN_AK_READ;
    FILTER0 = FILTER0_ALL_LLC;
    FILTER1 = FILTER1_OFF; /// should remain off on my tests.
    logger->info("COUNTER_CONTROL0: {}", descriptions[COUNTER_CONTROL0]);
    logger->info("COUNTER_CONTROL1: {}", descriptions[COUNTER_CONTROL1]);
    logger->info("COUNTER_CONTROL2: {}", descriptions[COUNTER_CONTROL2]);
    logger->info("COUNTER_CONTROL3: {}", descriptions[COUNTER_CONTROL3]);
    logger->info("FILTER0: {}", descriptions[FILTER0]);
    logger->info("FILTER1: {}", descriptions[FILTER1]);

    /// setting up msr registers to measure traffic.
    vals = {COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3, FILTER0, FILTER1};
    setAllUncoreRegisters(vals);

    vals = {COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3}; /// I WONT READ FILTERS!
    
    auto duration = runCommMicroBenchmark(data, iteration_count, main_thread_core_id, secondary_thread_core_id, vals, msr_fds);
    logger->info("COUNTER_CONTROL0: {}", descriptions[COUNTER_CONTROL0]);
    logger->info("COUNTER_CONTROL1: {}", descriptions[COUNTER_CONTROL1]);
    logger->info("COUNTER_CONTROL2: {}", descriptions[COUNTER_CONTROL2]);
    logger->info("COUNTER_CONTROL3: {}", descriptions[COUNTER_CONTROL3]);
    logger->info("FILTER0: {}", descriptions[FILTER0]);
    logger->info("FILTER1: {}", descriptions[FILTER1]);

    logger->info("core: {}, core: {}", main_thread_core_id, secondary_thread_core_id);
    logger->info("cha: {}", assigned_cha);
    logger->info("iteration_count: {}", iteration_count);
    logger->info("total hop: {}", total_hop);

    logger->info("================================================ ENDING ================================================");
    delete logger;
    logger = nullptr;
    
    return 0;
}

/// returns assigned cha of allocated memory.
int runMicroBenchmark(std::map<int, int>& msr_fds, const std::vector<unsigned int>& vals, long long* data, long long iteration_count, bool flush)
{
    logger->debug("-------------------------------------------------------------- MICROBENCHMARK -------------------------------------------------------------------");
    logger->debug("will flush?: {}", flush ? "yes" : "no");
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
            _mm_mfence();
            _mm_clflush(&data[0]);
            _mm_mfence();
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

    logger->debug("---------------- ANALYZING ----------------");
    long long sum_diff0 = 0;
    long long sum_diff1 = 0;
    long long sum_diff2 = 0;
    long long sum_diff3 = 0;

    for(const auto& read_val : read_vals) {
        logger->debug("map entry -> {} : <{} {} {} {}, {} {} {} {}>", read_val.first, 
        read_val.second.first[0], read_val.second.first[1], read_val.second.first[2], read_val.second.first[3], 
        read_val.second.second[0], read_val.second.second[1], read_val.second.second[2], read_val.second.second[3]);

        logger->debug("CHA: {}, read diff: <{} {} {} {}>", read_val.first, 
                                                    read_val.second.second[0] - read_val.second.first[0],
                                                    read_val.second.second[1] - read_val.second.first[1],
                                                    read_val.second.second[2] - read_val.second.first[2],
                                                    read_val.second.second[3] - read_val.second.first[3]);

        sum_diff0 += (read_val.second.second[0] - read_val.second.first[0]);
        sum_diff1 += (read_val.second.second[1] - read_val.second.first[1]);
        sum_diff2 += (read_val.second.second[2] - read_val.second.first[2]);
        sum_diff3 += (read_val.second.second[3] - read_val.second.first[3]);
    }

    int assigned_cha = -1;
    double data_read_max_percentage = 0;

    logger->info("Summarizing in terms of percentage:");
    for(const auto& read_val : read_vals) {
        logger->info("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", read_val.first, 
        ((read_val.second.second[0] - read_val.second.first[0]) / (double)sum_diff0) * 100,
        ((read_val.second.second[1] - read_val.second.first[1]) / (double)sum_diff1) * 100,
        ((read_val.second.second[2] - read_val.second.first[2]) / (double)sum_diff2) * 100,
        ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100);

        if(((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100 > data_read_max_percentage) {
            data_read_max_percentage = ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100;
            assigned_cha = read_val.first;
        }
    }

    return assigned_cha;
}

void incrementLoop(long long* data, long long iteration_count, int core_id)
{
    stick_this_thread_to_core(core_id);	
    // logger->info("Core: {}", sched_getcpu());
    // trash_logger->info("Starting... Core: {}", sched_getcpu());

    for(long long i = 0; i < iteration_count; ++i) {
        unique_lock<mutex> ul(g_val_mu);
        cv.wait(ul, []{return write_;});
        ++data[0];
        _mm_mfence();
        _mm_clflush(&data[0]);
        _mm_mfence();
        
        thread_modify_orders_.push_back(sched_getcpu());
        write_ = false;
        read_ = true;

        // trash_logger->debug("Core: {}", sched_getcpu());

        // Manual unlocking is done before notifying, to avoid waking up
        // the waiting thread only to block again (see notify_one for details)
        ul.unlock();
        cv.notify_one();
    }
}

double runCommMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const std::vector<unsigned int>& vals, std::map<int, int>& msr_fds)
{
	logger->debug(">>>>>>>>>>>>>>>>>>>>>");
	logger->debug("Microbenchmark started!");

    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> read_vals;

    stick_this_thread_to_core(main_thread_core_id); /// actually we did this at startup, just to make sure, do it again!

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


    // std::atomic<long long> dat {0};
    // iteration_count = 1'000;

    std::thread other_thread(incrementLoop, data, iteration_count, secondary_thread_core_id);
    sleep(1); /// make sure that thread is created and waiting on cv.

    // auto start = rdtsc();

    for(long long i = 0; i < iteration_count; ++i) {
        {
            unique_lock<mutex> ul(g_val_mu);
            ++data[0];
            _mm_mfence();
            _mm_clflush(&data[0]);
            _mm_mfence();
            thread_modify_orders_.push_back(sched_getcpu());
            write_ = true;
            read_ = false;
        }

        cv.notify_one();

        {
            std::unique_lock<std::mutex> ul(g_val_mu);
            cv.wait(ul, []{return read_;});
        }
    }

    other_thread.join();

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

    logger->debug("---------------- ANALYZING ----------------");
    long long sum_diff0 = 0;
    long long sum_diff1 = 0;
    long long sum_diff2 = 0;
    long long sum_diff3 = 0;

    for(const auto& read_val : read_vals) {
        logger->debug("map entry -> {} : <{} {} {} {}, {} {} {} {}>", read_val.first, 
        read_val.second.first[0], read_val.second.first[1], read_val.second.first[2], read_val.second.first[3], 
        read_val.second.second[0], read_val.second.second[1], read_val.second.second[2], read_val.second.second[3]);

        logger->debug("CHA: {}, read diff: <{} {} {} {}>", read_val.first, 
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

    /// val-cha pairs.
    std::set<std::pair<double, int>, std::greater<>> left_vals;
    std::set<std::pair<double, int>, std::greater<>> right_vals;
    std::set<std::pair<double, int>, std::greater<>> up_vals;
    std::set<std::pair<double, int>, std::greater<>> down_vals;
    std::set<std::pair<double, int>, std::greater<>> horizontal_vals;
    std::set<std::pair<double, int>, std::greater<>> vertical_vals;
    std::set<std::pair<double, int>, std::greater<>> total_vals;
    for(const auto& read_val : read_vals) {
        auto cha   = read_val.first;
        auto left  = ((read_val.second.second[0] - read_val.second.first[0]) / (double)sum_diff0) * 100;
        auto right = ((read_val.second.second[1] - read_val.second.first[1]) / (double)sum_diff1) * 100;
        auto up    = ((read_val.second.second[2] - read_val.second.first[2]) / (double)sum_diff2) * 100;
        auto down  = ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100;

        logger->info("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", cha, left, right, up, down);

        left_vals.insert({left, cha});
        right_vals.insert({right, cha});
        up_vals.insert({up, cha});
        down_vals.insert({down, cha});

        horizontal_vals.insert({left + right, cha});
        vertical_vals.insert({up + down, cha});
        total_vals.insert({left + right + up + down, cha});
    }

    logger->info("---\nAnalyzing left traffic:");
    for(const auto left_val : left_vals) {
        logger->info("cha: {}, percentage: {}", left_val.second, left_val.first);
    }
    logger->info("---\nAnalyzing right traffic:");
    for(const auto right_val : right_vals) {
        logger->info("cha: {}, percentage: {}", right_val.second, right_val.first);
    }
    logger->info("---\nAnalyzing up traffic:");
    for(const auto up_val : up_vals) {
        logger->info("cha: {}, percentage: {}", up_val.second, up_val.first);
    }
    logger->info("---\nAnalyzing down traffic:");
    for(const auto down_val : down_vals) {
        logger->info("cha: {}, percentage: {}", down_val.second, down_val.first);
    }
    logger->info("---\nAnalyzing vertical traffic:");
    for(const auto vertical_val : vertical_vals) {
        logger->info("cha: {}, percentage: {}", vertical_val.second, vertical_val.first);
    }
    logger->info("---\nAnalyzing horizontal traffic:");
    for(const auto horizontal_val : horizontal_vals) {
        logger->info("cha: {}, percentage: {}", horizontal_val.second, horizontal_val.first);
    }
    logger->info("---\nAnalyzing total traffic:");
    for(const auto total_val : total_vals) {
        logger->info("cha: {}, percentage: {}", total_val.second, total_val.first);
    }


    // auto end = rdtsc();
    // auto diff_tsc = end - start;

    // logger->info("Start-to-end tsc: {}", diff_tsc);
    logger->info("data last value: {}", data[0]);
    logger->info("allocated virtual data address: {:p}", (void*)&data[0]);

    bool is_broken = false;
    for(int i = 1; i < thread_modify_orders_.size(); ++i) {
        if(thread_modify_orders_[i] == thread_modify_orders_[i - 1]) {
            logger->error("Thread modify order was broken!");
            is_broken = true;
            break;
        }
    }

    if(!is_broken){
        logger->info("Thread modify order is correct.");
    }

    logger->debug("Microbenchmark ended.");
	logger->debug("<<<<<<<<<<<<<<<<<<<<<");

    return 0;
}

