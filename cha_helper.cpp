#include "cha_helper.h"
#include "logger_helper.h"
#include "cpu_helper.h"
#include "intel_helper.h" // for x86intrin.h
#include "thread_helper.h" // for stick_this_thread_to_core()
#include "os_helper.h"
#include "constants.h"

int compute_perm(long physical_address)
{
    long SelectorMasks[14] = {0x4c8fc0000, 0x1d05380000, 0x262b8c0000, 0x41f500000, 0x2c6d780000,
                              0x2cd5140000, 0x21d80c0000, 0x3b3f480000, 0x3a03500000, 0x3033280000,
                              0x0, 0x1469b40000, 0x0, 0x0};
    long i,j,k;
    int computed_perm = 0;
 
    for (int bit=0; bit<14; bit++) {
        auto permutation_selector_mask = SelectorMasks[bit];
        logger->debug("Selector mask: 0b{:b}; 0x{:x}", permutation_selector_mask, permutation_selector_mask);

        k = permutation_selector_mask & physical_address; // bitwise AND with mask
        logger->debug("will AND below 2 numbers.");
        logger->debug("0b{:064b} (0x{:016x})", permutation_selector_mask, permutation_selector_mask);
        logger->debug("0b{:064b} (0x{:016x})", physical_address, physical_address);
        logger->debug("0b{:064b} (0x{:016x}) -> AND (&) result.", k, k);
        
        j = __builtin_popcountl(k); // count number of bits set
        logger->debug("Number of bits in 0b{:b} : {}", k, j);

        i = j % 2; // compute parity
        logger->debug("Parity of 0b{:b}: {}", j, i);
        
        computed_perm += (i << bit); // scale and accumulate
        logger->debug("computed permutation += Parity ({}) << {} --> 0b{:b}", i, bit, computed_perm);
    }

    logger->debug("Computed permutation for physical address 0x{:x}: {}", physical_address, computed_perm);

    return (computed_perm); /// why parentheses around variable here?
}

/// returns assigned cha of allocated memory. USES PERFORMANCE COUNTERS!
/// second param: which index of the first param (array) will be used. default is set 0 in declaration. (btw all indexes that map to same cache line will be mapped to same CHA.)
int findCHA(long long* data, int index)
{
    logger->debug("-------------------------------------------------------------- FINDCHA STARTED -------------------------------------------------------------------");
    logger->debug("index: {}", index);

    stick_this_thread_to_core(17); /// stick thread to any core. --> made this 17 from 0 so that I dont bind to an isolated core. isolated cores are 0-15 last time I checked. 

    // const long long iteration_count = 300'000'000; --> this is safe, but time consuming.
    const long long iteration_count = 30'000'000; // --> seems to be working as well, takes significantly lower time (~5s). 

    logger->debug("number of iterations for flush step: {} million.", iteration_count / 1'000'000);
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

    logger->debug("Modifying data and then flushing immediately several times. This should take long.");

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(long long i = 0; i < iteration_count; ++i) {
        data[index] += 1;
        _mm_mfence();
        _mm_clflush(&data[index]);
        _mm_mfence();
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();  

    logger->debug("Flush step took {} milliseconds.", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());      

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

    logger->debug("Weight of the most LLC lookuped CHA amongst all CHAs: {}", data_read_max_percentage);
    logger->debug("assigned cha of address {:p}: {}", static_cast<void*>(&data[index]), assigned_cha);

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

/// it is important to get the pointer by reference so that we do not copy it here! Has trouble while working with space allocated by mmap().
int findCHAByHashing(long long*& val)
{
    // logger->info("findCHAByHashing input pointer: {}", (void*)val);

    pid_t pid = getpid();

    uintptr_t physical_address = 0;

    uintptr_t virtual_address = reinterpret_cast<uintptr_t>(&val);
    // logger->info("virtual_address: 0x{:x}", virtual_address);

    if (virt_to_phys_user(&physical_address, pid, virtual_address)) {
        logger->error("error: virt_to_phys_user");
        return EXIT_FAILURE;
    };
        
    logger->debug("virtual address 0x{:x} is mapped to physical address 0x{:x}.", virtual_address, physical_address);

    auto computed_perm = compute_perm(physical_address);

    auto physical_address_index = getIndex(physical_address);

    auto base_sequence_index = computed_perm ^ physical_address_index; /// XOR'ing.
    

    std::vector<int> base_sequence = readBaseSequence("BaseSequence_SKX_18-slice.txt");
    logger->debug("base_sequence_index: {}", base_sequence_index);

    assert(base_sequence_index < 4096 && "Base sequence must be lower than 4096!");
    int cha_by_hashing = base_sequence[base_sequence_index];
    logger->debug("CHA of virtual address 0x:{:x} using hashing method is {}", virtual_address, cha_by_hashing);

    return cha_by_hashing;
}

void runDifferentChaFindMethodsToCompare()
{
    std::atomic<long long> var{10};
    long long* var_ptr = reinterpret_cast<long long*>(&var);

    logger->info("var_ptr: {}", (void*)var_ptr);
    
    int cha_by_hashing = findCHAByHashing(var_ptr);


    int cha_by_perf_counters = findCHA(var_ptr);
    
    if(cha_by_hashing == cha_by_perf_counters) {
        logger->info("CHAs are equal as it is expected! They are both {}.", cha_by_perf_counters);
    } else {
        logger->error("CHAs are not equal! perf: {}, hashing: {}", cha_by_perf_counters, cha_by_hashing);
    }
}