#include "thread_helper.h"
#include "intel_helper.h"
#include "logger_helper.h"
#include "cpu_helper.h"
#include "cha_helper.h"

#include "constants.h"

#include <x86intrin.h> /// _mm_clflush

#include <sched.h>
#include <algorithm>

std::shared_ptr<spdlog::logger> logger = nullptr;

#include <set>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <fstream>
#include <vector>
#include <condition_variable>
#include <csignal>
#include <atomic>
#include <climits>
#include <pthread.h>
#include <omp.h>

#include <sys/mman.h>
#include <fcntl.h>

#define _XOPEN_SOURCE 700
#define CACHELINE_SZ (64) // aditya

#include <fcntl.h> /* open */
#include <stdint.h> /* uint64_t  */
#include <stdio.h> /* printf */
#include <stdlib.h> /* size_t */
#include <unistd.h> /* pread, sysconf */


#include "os_helper.h"
#include "imc_helper.h"
#include "pci_helper.h"

using namespace std;

/// aditya
struct SharedData_t{
	volatile uint64_t dummy1[CACHELINE_SZ/sizeof(uint64_t)]; // avoid buddy cacheline prefetching
	//volatile uint64_t data[CACHELINE_SZ/sizeof(uint64_t)]; 
	atomic<uint64_t> data[CACHELINE_SZ/sizeof(uint64_t)]; 
	volatile uint64_t dummy2[CACHELINE_SZ/sizeof(uint64_t)]; // avoid buddy cacheline prefetching
}__attribute__((aligned(64)));

SharedData_t trueSharingData;

int runAdityaBenchmark();
/// aditya

pthread_spinlock_t g_lock;
std::atomic<bool> thread_started;

double g_chunk[100'000'000] = {0.0};

unsigned int *mmconfig_ptr;         // must be pointer to 32-bit int so compiler will generate 32-bit loads and stores

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

unsigned long long runPingPongMicroBenchmark(long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param, std::unordered_map<int, long long*>& addresses);
void readTrafficCountersWhileReadingData(int core);
// void runPingPMABenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param); make sure that topo is correct by using PMA_GV perf counter!
static inline void incrementLoop(std::atomic<long long>& data, long long iteration_count, int core_id);

int main(int argc, char** argv)
{
    ignoreHangupSignal();   
    
    setLogger("logs/refactored_monday.txt", spdlog::level::info);
    assertRoot(); /// assert root must come after setLogger() since it uses logger instance.
    logger->info("================================================ STARTING ================================================");

    std::set<int> encountered_chas;
    std::unordered_map<int, long long*> addresses;

    /// this needs better data structure organization: no need for set when I have unordered_map!
    /// 65536 is quite a large chunk, 64 could do it: a page consists of 64 cache lines in this system.
    while(encountered_chas.size() < 18) {
        auto addr = static_cast<long long*>(allocateChunk(65536));
        logger->info("alloc addr: {}", (void*)addr);
        // *addr = 15;
        const int perf_cha = findCHA(addr); /// hashing method did not work here somehow!
        
        if(encountered_chas.insert(perf_cha).second) {
            logger->info("encountered new cha: {}", perf_cha);

            for(auto encountered_cha : encountered_chas) {
                logger->info("encountered cha: {}", encountered_cha);
            }

            logger->info("addr: {}", (void*)addr);
            addresses[perf_cha] = addr;
        } else {
            logger->info("discarded cha: {}", perf_cha);
            // logger->info("freed addr: {}", (void*)addr);
            // std::free(addr); --> it is important not to free, otherwise in the next iteration os will allocate this freed space again, causing a livelock.
        }
    }

    logger->info("All chas encountered!!!!!!!!!!!!!!!!!!!!!!!");
    // return 0;

    // std::unordered_map<int, std::vector<long long*>> vals;

    // /// example address: 0x7fff66f5ebe8, 48 bits of virtual address in Linux.

    // // 0x30cabdefa612, 0x66cabdefa612, 0x48cabdefa612, 0x96cabdefa612
    // std::vector<void*> addrs;


    // /// these addresses have to be multiple of 4096 (page size of 4KB), because they will be used as a first argument to mmap() which requires address this way.
    // addrs.push_back((void*)0x30abcdefa000);
    // addrs.push_back((void*)0x660123456000);
    // addrs.push_back((void*)0x489876543000);
    // addrs.push_back((void*)0x7f96eab26000);

    // for(auto& addr : addrs) {
    // //            EINVAL We don't like addr, length, or offset (e.g., they are too
    // //           large, or not aligned on a page boundary).

    // //    EINVAL (since Linux 2.6.12) length was 0.

    // //    EINVAL flags contained none of MAP_PRIVATE, MAP_SHARED, or -----> THIS IS IMPORTANT
    // //           MAP_SHARED_VALIDATE.

    //     logger->info("addr: {}", addr);
    //     /// since I did not specify MAP_FIXED, exact address is not guaranteed to be given by OS to me.
    //     long long* mapped_addr = static_cast<long long*>(mmap(addr, sizeof(long long), PROT_READ | PROT_WRITE | PROT_EXEC, MAP_ANONYMOUS | MAP_SHARED, -1, 0));

    //     *mapped_addr = 10; /// just to touch it, just in case.

    //     // std::cout << *mapped_addr << std::endl;
    //     // return 0;

    //     if(addr == MAP_FAILED) {
    //         std::cout << "MAPPING FAILED: " << strerror(errno) << std::endl;
    //         logger->info("MAPPING FAILED");
    //         continue;    
    //     }

    //     logger->info("mapped_addr: {}", (void*)mapped_addr);

    //     const int perf_cha = findCHA(mapped_addr);
    //     logger->info("perf cha: {}", perf_cha);

    //     // const int hash_cha = findCHAByHashing(mapped_addr);  // troublesome while working with space allocated by mmap().
    //     // logger->info("hash cha: {}", hash_cha);
    // }

    // logger->info("Summary");
    // for(const auto& pp : vals) {
    //     logger->info("{} -> {} items.", pp.first, pp.second.size());
    // }
    

    // return 0;


    // runAdityaBenchmark();

    // return 0;


    const long long ping_pong_iteration_count = 1'000'000'000;
    logger->debug("number of ping pong iterations : {} million.", ping_pong_iteration_count / 1'000'000);

    std::vector<std::pair<int, int>> core_pairs{{0, 14}, {0, 9}, {7, 5}, {0, 15}, {17, 15}, {0, 17}, {10, 6}, {0, 12}, {13, 3}, {10, 2}, {14, 8}, {8, 13}, {8, 16}, {3, 11}, {11, 5}, {1, 5}, {1, 9}};
    
    for(const auto& core_pair : core_pairs){
        for(int i = 0; i < 5; ++i) {
            auto first_core  = core_pair.first;
            auto second_core = core_pair.second;

            // void* cache_line = allocateCacheLine();
            // auto data = static_cast<long long*>(cache_line);
            // g_assigned_cha = findCHA(data);

            {
            ConfigParam config_param{LEFT_BL_READ, RIGHT_BL_READ, UP_BL_READ, DOWN_BL_READ, FILTER0_OFF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(ping_pong_iteration_count, first_core, second_core, config_param, addresses);
            }

            {
            ConfigParam config_param{LEFT_AD_READ, RIGHT_AD_READ, UP_AD_READ, DOWN_AD_READ, FILTER0_OFF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(ping_pong_iteration_count, first_core, second_core, config_param, addresses);
            }

            // {
            // ConfigParam config_param{LEFT_AK_READ, RIGHT_AK_READ, UP_AK_READ, DOWN_AK_READ, FILTER0_OFF, FILTER1_OFF};
            // auto duration = runPingPongMicroBenchmark(ping_pong_iteration_count, first_core, second_core, config_param);
            // }

            // {
            // ConfigParam config_param{LEFT_IV_READ, RIGHT_IV_READ, UP_IV_READ, DOWN_IV_READ, FILTER0_OFF, FILTER1_OFF};
            // auto duration = runPingPongMicroBenchmark(ping_pong_iteration_count, first_core, second_core, config_param);
            // }

            // {
            // ConfigParam config_param{HITME_LOOKUP_READ, HITME_LOOKUP_WRITE, LLC_DATA_READ_LOOKUP, LLC_WRITE_LOOKUP, FILTER0_ALL_COMBINED_LLC_SF, FILTER1_OFF};
            // auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            // }
            /// MAKE SURE THAT I FOUND CORES RIGHT -> MEASURE PMA_GV.
            /// ALIGNAS dene. 
        }
    }

/*
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
*/
}

static inline void incrementLoop(std::atomic<long long>& data, long long iteration_count, int core_id)
{
    thread_started = true;

    stick_this_thread_to_core(core_id);	
    // logger->debug("Core {} modifying data {}, {} times.", core_id, static_cast<void*>(data), iteration_count);

    for(long long i = 0; i < iteration_count; ++i) {
        ++data;
    }
}

static inline void incrementLoop2(long long& data, long long iteration_count, int core_id)
{
    thread_started = true;

    stick_this_thread_to_core(core_id);	
    // logger->debug("Core {} modifying data {}, {} times.", core_id, static_cast<void*>(data), iteration_count);

    for(long long i = 0; i < iteration_count; ++i) {
        ++data;
    }
}


unsigned long long runPingPongMicroBenchmark(long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param, std::unordered_map<int, long long*>& addresses)
{
    // long long var{0};
    // long long* data = reinterpret_cast<long long*>(&var);
    // int assigned_cha = findCHAByHashing(data);


    for(auto& pp : addresses) {
        int assigned_cha = pp.first;
        long long* data = pp.second;
        long long var = *data;

        std::vector<unsigned int> vals = {config_param.CTR0, config_param.CTR1, config_param.CTR2, config_param.CTR3, config_param.FIL0, config_param.FIL1};
        setAllUncoreRegisters(vals);
        vals = {config_param.CTR0, config_param.CTR1, config_param.CTR2, config_param.CTR3};

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
        logger->info("assigned cha: {}", assigned_cha);

        std::thread other_thread(incrementLoop2, std::ref(var), iteration_count, secondary_thread_core_id);
        while(!thread_started)
        ;

        // logger->info("---");
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        incrementLoop2(var, iteration_count, main_thread_core_id);
        other_thread.join();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        unsigned long long elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        // logger->info("PingPong duration in seconds: ~{}.", std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());
        if(var == 2 * iteration_count) {
            logger->info("Threadsafe increment!");
        } else {
            logger->error("Variable has not expected value!");
        }

        logger->info("PingPongDuration: {} ms, core: {}, core: {}, cha: {}, iteration_count: {}, variable: {}", elapsed_time, main_thread_core_id, secondary_thread_core_id, assigned_cha, iteration_count, var);
        logger->info("---");
        
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

        logger->debug("Summarizing in terms of percentage:");
        /// val-cha pairs.
        std::set<std::pair<double, int>, std::greater<>> val0s;
        std::set<std::pair<double, int>, std::greater<>> val1s;
        std::set<std::pair<double, int>, std::greater<>> val2s;
        std::set<std::pair<double, int>, std::greater<>> val3s;
        // std::set<std::pair<double, int>, std::greater<>> horizontal_vals;
        // std::set<std::pair<double, int>, std::greater<>> vertical_vals;
        // std::set<std::pair<double, int>, std::greater<>> total_vals;

        for(const auto& read_val : read_vals) {
            auto cha   = read_val.first;
            auto var0 = ((read_val.second.second[0] - read_val.second.first[0]) / (double)sum_diff0) * 100;
            auto var1 = ((read_val.second.second[1] - read_val.second.first[1]) / (double)sum_diff1) * 100;
            auto var2 = ((read_val.second.second[2] - read_val.second.first[2]) / (double)sum_diff2) * 100;
            auto var3 = ((read_val.second.second[3] - read_val.second.first[3]) / (double)sum_diff3) * 100;

            logger->debug("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", cha, var0, var1, var2, var3);

            val0s.insert({var0, cha});
            val1s.insert({var1, cha});
            val2s.insert({var2, cha});
            val3s.insert({var3, cha});

            // horizontal_vals.insert({left + right, cha});
            // vertical_vals.insert({up + down, cha});
            // total_vals.insert({left + right + up + down, cha});
        }

        logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR0], main_thread_core_id, secondary_thread_core_id, assigned_cha);
        for(const auto val0 : val0s) {
            logger->info("cha: {}, percentage: {}", val0.second, val0.first);
        }
        logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR1], main_thread_core_id, secondary_thread_core_id, assigned_cha);
        for(const auto val1 : val1s) {
            logger->info("cha: {}, percentage: {}", val1.second, val1.first);
        }
        logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR2], main_thread_core_id, secondary_thread_core_id, assigned_cha);
        for(const auto val2 : val2s) {
            logger->info("cha: {}, percentage: {}", val2.second, val2.first);
        }
        logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR3], main_thread_core_id, secondary_thread_core_id, assigned_cha);
        for(const auto val3 : val3s) {
            logger->info("cha: {}, percentage: {}", val3.second, val3.first);
        }
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

        logger->debug("closing file descriptors of MSRs.");
        for(const auto& p : msr_fds) {
            int cpu = p.first;
            int to_be_closed = p.second;
            logger->debug("closing fd {} of cpu {}.", to_be_closed, cpu);
            ::close(to_be_closed);
        }

        // logger->info("Cache line transfer microbenchmark ended.");
        // logger->info("<<<<<<<<<<<<<<<<<<<<<");

        // return elapsed_time;
    }
    
    return 0;
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

int runAdityaBenchmark() {
	atomic<uint64_t> justToAvoidCompilerOptimization;

    atomic<uint64_t> *tsData = &trueSharingData.data[0];
    auto tsData_ptr = reinterpret_cast<long long*>(tsData);
    auto cha = findCHAByHashing(tsData_ptr);

    logger->info("aditya benchmark cha: {}", cha);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	#pragma omp parallel num_threads(2)
	{	
        int bound_core_no = sched_getcpu();
        logger->info("running on core: {}", bound_core_no);

        atomic<uint64_t> *tsData = &trueSharingData.data[0];
        
        for(uint64_t i = 0 ; i < 100'000'000; i++) {
            __asm__ __volatile__ ("MFENCE;");
            *tsData += 1;
        }

        justToAvoidCompilerOptimization += *tsData;	
	}

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    unsigned long long elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    logger->info("aditya benchmark elapsed time: {} ms", elapsed_time);

	return justToAvoidCompilerOptimization.load() ^ justToAvoidCompilerOptimization.load();
}

