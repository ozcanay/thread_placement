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

double g_chunk[100'000'000] = {0.0};


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

struct ConfigParam
{
    unsigned int CTR0;
    unsigned int CTR1;
    unsigned int CTR2;
    unsigned int CTR3;
    unsigned int FIL0;
    unsigned int FIL1;
};

int findCHA(long long* data);
unsigned long long runPingPongMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param);
void readTrafficCountersWhileReadingData(int core);
// void runPingPMABenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param); make sure that topo is correct by using PMA_GV perf counter!
void incrementLoop(long long* data, long long iteration_count, int core_id);

int g_assigned_cha = -1;

int main(int argc, char** argv)
{
    std::signal(SIGHUP, SIG_IGN);
    setLogger("logs/traffic_measurement.txt");
    logger->info("================================================ STARTING ================================================");

    for(int i = 0; i < getCoreCount(); ++i) {
        readTrafficCountersWhileReadingData(i);
        sleep(1);
    }

    return 0;




    if(argc != 3) {
        std::cerr << "./placer <CORE_NO> <CORE_NO>\n";
        return 0;
    }

    thread_modify_orders_.reserve(40000000);
    std::signal(SIGHUP, SIG_IGN);

    setLogger("logs/core_pair_latencies.txt");
    logger->info("================================================ STARTING ================================================");
    
    int main_thread_core_id = std::stoi(argv[1]);
    int secondary_thread_core_id = std::stoi(argv[2]);;

    stick_this_thread_to_core(main_thread_core_id);


    int num_elements = CACHE_LINE_SIZE / sizeof(long long);
    logger->debug("num elements: {}", num_elements);
    void *v_data = nullptr;
    /// allocate heap of size equal to just 1 cache line.
    posix_memalign(&v_data, CACHE_LINE_SIZE, num_elements * sizeof(long long)); /// also try alignas 64 here from c++.
    long long* data = (long long*)v_data;
    logger->info("allocated virtual data address: {:p}", v_data);

    g_assigned_cha = findCHA(data);

    const long long ping_pong_iteration_count = 50'000'000;
    logger->info("number of ping pong iterations : {} million.", ping_pong_iteration_count / 1'000'000);

    // std::vector<std::pair<int, int>> core_pairs{{0, 14}, {14, 8}, {8, 13}, {13, 3}, {10, 2}, {7,5}, {5,1}, {1,9}, {17, 15}, {10, 6}, {8,16}, {3, 11}, {11,5}};
    // std::vector<std::pair<int, int>> core_pairs{{14, 8}};
    std::vector<std::pair<int, int>> core_pairs{{0, 14}, {7, 5}, {17, 15}, {10, 6}, {13, 3}};
    for(const auto& core_pair : core_pairs){
        for(int i = 0; i < 10; ++i) {
            auto first_core  = core_pair.first;
            auto second_core = core_pair.second;
            ConfigParam config_param{HITME_LOOKUP_READ, HITME_LOOKUP_WRITE, HITME_LOOKUP_ALL, HITME_LOOKUP_ALL, FILTER0_ALL_LLC, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            logger->critical("#{} PingPongDuration: {} ns, core: {}, core: {}, cha: {}, iteration_count: {}", i+1, duration, first_core, second_core, g_assigned_cha, ping_pong_iteration_count);
            logger->flush();
            /// MAKE SURE THAT I FOUND CORES RIGHT -> MEASURE PMA_GV.
            /// ALIGNAS dene. 
        }
    }

    // {
    //     ConfigParam config_param{HITME_HIT_EX_RDS, HITME_HIT_SHARED_OWNREQ, HITME_HIT_WBMTOE, HITME_HIT_WBMTOI_OR_S, FILTER0_ALL_LLC, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }

    // {
    //     ConfigParam config_param{HITME_MISS_SHARED_RDINVOWN, HITME_MISS_NOTSHARED_RDINVOWN, HITME_MISS_READ_OR_INV, HITME_MISS_ALL, FILTER0_ALL_LLC, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }

    // {
    //     ConfigParam config_param{HITME_UPDATE_DEALLOCATE_RSPFWDI_LOC, HITME_UPDATE_RSPFWDI_REM, HITME_UPDATE_SHARED, HITME_UPDATE_RDINVOWN, FILTER0_ALL_LLC, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    //     config_param = {HITME_UPDATE_DEALLOCATE, HITME_UPDATE_DEALLOCATE, HITME_UPDATE_DEALLOCATE, HITME_UPDATE_DEALLOCATE, FILTER0_ALL_LLC, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }

    // {
    //     ConfigParam config_param{LEFT_BL_READ, RIGHT_BL_READ, UP_BL_READ, DOWN_BL_READ, FILTER0_OFF, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }

    // {
    //     ConfigParam config_param{LEFT_AD_READ, RIGHT_AD_READ, UP_AD_READ, DOWN_AD_READ, FILTER0_OFF, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }

    // {
    //     ConfigParam config_param{LEFT_IV_READ, RIGHT_IV_READ, UP_IV_READ, DOWN_IV_READ, FILTER0_OFF, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }
    
    // {
    //     ConfigParam config_param{LEFT_AK_READ, RIGHT_AK_READ, UP_AK_READ, DOWN_AK_READ, FILTER0_OFF, FILTER1_OFF};
    //     runPingPongMicroBenchmark(data, ping_pong_iteration_count, main_thread_core_id, secondary_thread_core_id, config_param);
    // }
    
    logger->info("================================================ ENDING ================================================");
    delete logger;
    logger = nullptr;
    
    return 0;
}

/// returns assigned cha of allocated memory.
int findCHA(long long* data)
{
    logger->debug("-------------------------------------------------------------- FINDCHA STARTED -------------------------------------------------------------------");

    const long long iteration_count = 300'000'000;
    logger->info("number of iterations for flush step: {} million.", iteration_count / 1'000'000);
    auto msr_fds = getMsrFds();

    /// one of the counter control values would have sufficed but I do not want to modify setAllUncoreMethods function just for this purpose.
    unsigned int COUNTER_CONTROL0 = LLC_DATA_READ_LOOKUP; /// dummy
    unsigned int COUNTER_CONTROL1 = LLC_DATA_READ_LOOKUP; /// dummy
    unsigned int COUNTER_CONTROL2 = LLC_DATA_READ_LOOKUP; /// dummy
    unsigned int COUNTER_CONTROL3 = LLC_DATA_READ_LOOKUP; /// my logic relies on this counters being LLC_DATA_READ_LOOKUP, so do not change it here.
    unsigned int FILTER0 = FILTER0_ALL_LLC; /// important that filter0 takes this value since we will measure LLC lookup events.
    unsigned int FILTER1 = FILTER1_OFF; /// should remain off on my tests.

    std::vector<unsigned int> vals{COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3, FILTER0, FILTER1};
    setAllUncoreRegisters(vals);
    vals = {COUNTER_CONTROL0, COUNTER_CONTROL1, COUNTER_CONTROL2, COUNTER_CONTROL3}; /// I WONT READ FILTERS!

    /// key: cha || value: pair of vectors, each vector representing read counter value, each pair representing before and after values.
    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> read_vals;

    logger->debug("---------------- FIRST READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (CHA_BASE * cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].first.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                }
            }
        }
    }

    logger->info("Modifying data and then flushing immediately several times. This should take long.");

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(long long i = 0; i < iteration_count; ++i) {
        data[0] += 1;
        _mm_mfence();
        _mm_clflush(&data[0]);
        _mm_mfence();
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();  

    logger->info("Flush step took {} milliseconds.", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());      

    logger->debug("---------------- SECOND READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (CHA_BASE * cha) + i;
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

    logger->debug("Summarizing in terms of percentage:");
    for(const auto& read_val : read_vals) {
        logger->debug("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", read_val.first, 
        ((read_val.second.second[0] - read_val.second.first[0]) / (double)sum_diff0) * 100,
        ((read_val.second.second[1] - read_val.second.first[1]) / (double)sum_diff1) * 100,
        ((read_val.second.second[2] - read_val.second.first[2]) / (double)sum_diff2) * 100,
        ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100);

        if(((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100 > data_read_max_percentage) {
            data_read_max_percentage = ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100;
            assigned_cha = read_val.first;
        }
    }

    logger->debug("COUNTER_CONTROL0: {}", descriptions[COUNTER_CONTROL0]);
    logger->debug("COUNTER_CONTROL1: {}", descriptions[COUNTER_CONTROL1]);
    logger->debug("COUNTER_CONTROL2: {}", descriptions[COUNTER_CONTROL2]);
    logger->debug("COUNTER_CONTROL3: {}", descriptions[COUNTER_CONTROL3]);
    logger->debug("FILTER0: {}", descriptions[FILTER0]);
    logger->debug("FILTER1: {}", descriptions[FILTER1]);

    logger->info("Weight of the most LLC lookuped CHA amongst all CHAs: {}", data_read_max_percentage);
    logger->info("assigned cha of address {:p}: {}", static_cast<void*>(data), assigned_cha);

    logger->debug("closing file descriptors of MSRs.");
    for(const auto& p : msr_fds) {
        int cpu = p.first;
        int to_be_closed = p.second;
        logger->debug("closing fd {} of cpu {}.", to_be_closed, cpu);
        ::close(to_be_closed);
    }

    logger->debug("-------------------------------------------------------------- FINDCHA ENDED -------------------------------------------------------------------");

    return assigned_cha;
}

void incrementLoop(long long* data, long long iteration_count, int core_id)
{
    stick_this_thread_to_core(core_id);	

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

        // Manual unlocking is done before notifying, to avoid waking up
        // the waiting thread only to block again (see notify_one for details)
        ul.unlock();
        cv.notify_one();
    }
}

unsigned long long runPingPongMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param)
{
    logger->info("Cache line transfer microbenchmark started.");
    logger->info(">>>>>>>>>>>>>>>>>>>>>");

    std::vector<unsigned int> vals = {config_param.CTR0, config_param.CTR1, config_param.CTR2, config_param.CTR3, config_param.FIL0, config_param.FIL1};
    setAllUncoreRegisters(vals);

    /// I will not read filters!
    vals.pop_back();
    vals.pop_back();

    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> read_vals;

    auto msr_fds = getMsrFds();

    stick_this_thread_to_core(main_thread_core_id); /// actually we did this at startup, just to make sure, do it again!

    logger->debug("---------------- FIRST READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (CHA_BASE * cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].first.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                }
            }
        }
    }

    logger->info("COUNTER_CONTROL0: {}", descriptions[config_param.CTR0]);
    logger->info("COUNTER_CONTROL1: {}", descriptions[config_param.CTR1]);
    logger->info("COUNTER_CONTROL2: {}", descriptions[config_param.CTR2]);
    logger->info("COUNTER_CONTROL3: {}", descriptions[config_param.CTR3]);
    logger->info("FILTER0: {}", descriptions[config_param.FIL0]);
    logger->info("FILTER1: {}", descriptions[config_param.FIL1]);
    logger->info("core: {}, core: {}", main_thread_core_id, secondary_thread_core_id);
    logger->info("assigned cha: {}", g_assigned_cha);

    std::thread other_thread(incrementLoop, data, iteration_count, secondary_thread_core_id);
    sleep(1); /// make sure that "other_thread" is created and waiting on cv.

    logger->info("------------------------------ PingPong started ------------------------------");
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
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
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    unsigned long long return_value = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();

    logger->info("------------------------------ PingPong ended ------------------------------");
    logger->info("PingPong duration in seconds: ~{}.", std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());
    
    logger->debug("---------------- SECOND READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (CHA_BASE * cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].second.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
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
    // std::set<std::pair<double, int>, std::greater<>> left_vals;
    // std::set<std::pair<double, int>, std::greater<>> right_vals;
    // std::set<std::pair<double, int>, std::greater<>> up_vals;
    // std::set<std::pair<double, int>, std::greater<>> down_vals;
    // std::set<std::pair<double, int>, std::greater<>> horizontal_vals;
    // std::set<std::pair<double, int>, std::greater<>> vertical_vals;
    // std::set<std::pair<double, int>, std::greater<>> total_vals;

    for(const auto& read_val : read_vals) {
        auto cha   = read_val.first;
        auto left  = ((read_val.second.second[0] - read_val.second.first[0]) / (double)sum_diff0) * 100;
        auto right = ((read_val.second.second[1] - read_val.second.first[1]) / (double)sum_diff1) * 100;
        auto up    = ((read_val.second.second[2] - read_val.second.first[2]) / (double)sum_diff2) * 100;
        auto down  = ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100;

        logger->info("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", cha, left, right, up, down);

        // left_vals.insert({left, cha});
        // right_vals.insert({right, cha});
        // up_vals.insert({up, cha});
        // down_vals.insert({down, cha});

        // horizontal_vals.insert({left + right, cha});
        // vertical_vals.insert({up + down, cha});
        // total_vals.insert({left + right + up + down, cha});
    }

    // logger->info("---\nAnalyzing left traffic:");
    // for(const auto left_val : left_vals) {
    //     logger->info("cha: {}, percentage: {}", left_val.second, left_val.first);
    // }
    // logger->info("---\nAnalyzing right traffic:");
    // for(const auto right_val : right_vals) {
    //     logger->info("cha: {}, percentage: {}", right_val.second, right_val.first);
    // }
    // logger->info("---\nAnalyzing up traffic:");
    // for(const auto up_val : up_vals) {
    //     logger->info("cha: {}, percentage: {}", up_val.second, up_val.first);
    // }
    // logger->info("---\nAnalyzing down traffic:");
    // for(const auto down_val : down_vals) {
    //     logger->info("cha: {}, percentage: {}", down_val.second, down_val.first);
    // }
    // logger->info("---\nAnalyzing vertical traffic:");
    // for(const auto vertical_val : vertical_vals) {
    //     logger->info("cha: {}, percentage: {}", vertical_val.second, vertical_val.first);
    // }
    // logger->info("---\nAnalyzing horizontal traffic:");
    // for(const auto horizontal_val : horizontal_vals) {
    //     logger->info("cha: {}, percentage: {}", horizontal_val.second, horizontal_val.first);
    // }
    // logger->info("---\nAnalyzing total traffic:");
    // for(const auto total_val : total_vals) {
    //     logger->info("cha: {}, percentage: {}", total_val.second, total_val.first);
    // }

    logger->debug("data last value: {}", data[0]);
    logger->debug("allocated virtual address: {:p}", static_cast<void*>(&data[0]));

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

    logger->debug("closing file descriptors of MSRs.");
    for(const auto& p : msr_fds) {
        int cpu = p.first;
        int to_be_closed = p.second;
        logger->debug("closing fd {} of cpu {}.", to_be_closed, cpu);
        ::close(to_be_closed);
    }

    logger->info("Cache line transfer microbenchmark ended.");
	logger->info("<<<<<<<<<<<<<<<<<<<<<");

    return return_value;
}

void readTrafficCountersWhileReadingData(int core)
{
    logger->info(__PRETTY_FUNCTION__);

    stick_this_thread_to_core(core);

    auto msr_fds = getMsrFds();

    std::vector<unsigned int> vals = {LEFT_BL_READ, RIGHT_BL_READ, UP_BL_READ, DOWN_BL_READ, FILTER0_OFF, FILTER1_OFF};
    setAllUncoreRegisters(vals);

    /// popping up filter values since we will not read from them.
    vals.pop_back();
    vals.pop_back();

    /// key: cha || value: pair of vectors, each vector representing read counter value, each pair representing before and after values.
    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> read_vals;

    logger->debug("---------------- FIRST READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (CHA_BASE * cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].first.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                }
            }
        }
    }

    /// read huge data!
    logger->info("Starting fetching and updating data... This should take long.");
    for(auto& num : g_chunk) {
        num = num + 1.0;
    }
    logger->info("Finished updating data.");
    /// 

    logger->debug("---------------- SECOND READINGS ----------------");
    for(int socket = 0; socket < NUM_SOCKETS; ++socket) {
        long core = 0;

        for(int cha = 0; cha < NUM_CHA_BOXES; ++cha) {
            for(int i = 0; i < vals.size(); ++i) {
                uint64_t msr_val;
                uint64_t msr_num = CHA_MSR_PMON_CTR_BASE + (CHA_BASE * cha) + i;
                logger->debug("Executing pread() --> fd: {}, offset: {:x}", msr_fds[core], msr_num);
                ssize_t rc64 = pread(msr_fds[core], &msr_val, sizeof(msr_val), msr_num);
                if (rc64 != sizeof(msr_val)) {
                    logger->error("EXIT FAILURE. rc64: {}", rc64);
                    logger->error("error: {}", strerror(errno));
                    exit(EXIT_FAILURE);
                } else {
                    read_vals[cha].second.push_back(msr_val);
                    logger->debug("Read {} from socket {}, CHA {} on core {}, offset 0x{:x}.", msr_val, socket, cha, core, msr_num);
                }
            }
        }
    }


    /////////////////////////////// FINISHED WITH READINGS.


    logger->info("---------------- ANALYZING ----------------");
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

    logger->debug("Summarizing in terms of percentage:");

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

        logger->debug("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", cha, left, right, up, down);

        left_vals.insert({left, cha});
        right_vals.insert({right, cha});
        up_vals.insert({up, cha});
        down_vals.insert({down, cha});

        horizontal_vals.insert({left + right, cha});
        vertical_vals.insert({up + down, cha});
        total_vals.insert({left + right + up + down, cha});
    }

    logger->info("---\nAnalyzing left traffic (core{}left):", core);
    for(const auto left_val : left_vals) {
        logger->info("cha: {}, percentage: {}", left_val.second, left_val.first);
    }
    logger->info("---\nAnalyzing right traffic (core{}right):", core);
    for(const auto right_val : right_vals) {
        logger->info("cha: {}, percentage: {}", right_val.second, right_val.first);
    }
    logger->info("---\nAnalyzing up traffic (core{}up):", core);
    for(const auto up_val : up_vals) {
        logger->info("cha: {}, percentage: {}", up_val.second, up_val.first);
    }
    logger->info("---\nAnalyzing down traffic (core{}down):", core);
    for(const auto down_val : down_vals) {
        logger->info("cha: {}, percentage: {}", down_val.second, down_val.first);
    }
    logger->info("---\nAnalyzing vertical traffic (core{}vert):", core);
    for(const auto vertical_val : vertical_vals) {
        logger->info("cha: {}, percentage: {}", vertical_val.second, vertical_val.first / 2.0);
    }
    logger->info("---\nAnalyzing horizontal traffic (core{}horz):", core);
    for(const auto horizontal_val : horizontal_vals) {
        logger->info("cha: {}, percentage: {}", horizontal_val.second, horizontal_val.first / 2.0);
    }
    logger->info("---\nAnalyzing total traffic (core{}total):", core);
    for(const auto total_val : total_vals) {
        logger->info("cha: {}, percentage: {}", total_val.second, total_val.first / 4.0);
    }


    logger->debug("closing file descriptors of MSRs.");
    for(const auto& p : msr_fds) {
        int cpu = p.first;
        int to_be_closed = p.second;
        logger->debug("closing fd {} of cpu {}.", to_be_closed, cpu);
        ::close(to_be_closed);
    }
}

