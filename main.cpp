#include "thread_helper.h"
#include "intel_helper.h"
#include "logger_helper.h"
#include "cpu_helper.h"
#include "constants.h"

#include <x86intrin.h> /// _mm_clflush

#include <algorithm>

std::shared_ptr<spdlog::logger> logger = nullptr;

#include <set>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <fstream>
#include <condition_variable>
#include <csignal>
#include <atomic>
#include <climits>
#include <pthread.h>

#include <sys/mman.h>
#include <fcntl.h>

#define _XOPEN_SOURCE 700
#include <fcntl.h> /* open */
#include <stdint.h> /* uint64_t  */
#include <stdio.h> /* printf */
#include <stdlib.h> /* size_t */
#include <unistd.h> /* pread, sysconf */

using namespace std;

pthread_spinlock_t g_lock;
std::atomic<bool> thread_started;

double g_chunk[100'000'000] = {0.0};





int IMC_BUS_Socket[2] = {0x3a};
int IMC_Device_Channel[6] = {0x0a, 0x0a, 0x0b, 0x0c, 0x0c, 0x0d};
int IMC_Function_Channel[6] = {0x2, 0x6, 0x2, 0x2, 0x6, 0x2};
int IMC_PmonCtl_Offset[5] = {0xd8, 0xdc, 0xe0, 0xe4, 0xf0}; 
int IMC_PmonCtr_Offset[5] = {0xa0, 0xa8, 0xb0, 0xb8, 0xd0};

unsigned int *mmconfig_ptr;         // must be pointer to 32-bit int so compiler will generate 32-bit loads and stores





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

int findCHA(long long* data, int index = 0);
unsigned long long runPingPongMicroBenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param);
void readTrafficCountersWhileReadingData(int core);
// void runPingPMABenchmark(long long* data, long long iteration_count, int main_thread_core_id, int secondary_thread_core_id, const ConfigParam& config_param); make sure that topo is correct by using PMA_GV perf counter!
static inline void incrementLoop(long long* data, long long iteration_count, int core_id);

/// Test to see if memory address residing in the same cache line maps to the same CHA (it should.) 
void testSameCacheLineCHAAssignment();

int g_assigned_cha = -1;

int compute_perm(long physical_address)
{
    long SelectorMasks[14] = {0x4c8fc0000, 0x1d05380000, 0x262b8c0000, 0x41f500000, 0x2c6d780000,
                              0x2cd5140000, 0x21d80c0000, 0x3b3f480000, 0x3a03500000, 0x3033280000,
                              0x0, 0x1469b40000, 0x0, 0x0};
    long i,j,k;
    // long i,j,k;
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


void* allocateCacheLine()
{
    int num_elements = CACHE_LINE_SIZE / sizeof(long long);
    logger->debug("num elements: {}", num_elements);
    void *v_data = nullptr;
    /// allocate heap of size equal to just 1 cache line.
    posix_memalign(&v_data, CACHE_LINE_SIZE, num_elements * sizeof(long long)); /// also try alignas 64 here from c++.
    logger->info("allocated virtual data address: {:p}", v_data);

    return v_data;
}

void assertRoot()
{
    int uid = getuid();
    if (uid == 0) {
        logger->debug("Running as root.");
    } else {
        std::cout << "Not running as root. Need root privileges to run the app. Exiting." << std::endl;
        exit(EXIT_FAILURE);
    }
}

uint32_t PCI_cfg_index(unsigned int Bus, unsigned int Device, unsigned int Function, unsigned int Offset)
{
    uint32_t byteaddress;
    uint32_t index;
    // assert (Bus == BUS);
    // aydin: device -> 0-32, function -> 0-8,--> these are ok: https://en.wikipedia.org/wiki/PCI_configuration_space#:~:text=In%20addition%20to%20the%20normal,F%2C%20as%20abbreviated%20from%20bus
    // why: offset -> 0-4096?

    assert (Device >= 0);
    assert (Function >= 0);
    assert (Offset >= 0);
    assert (Device < (1<<5));
    assert (Function < (1<<3));
    assert (Offset < (1<<12));
#ifdef DEBUG
    fprintf(log_file,"Bus,(Bus<<20)=%x\n",Bus,(Bus<<20));
    fprintf(log_file,"Device,(Device<<15)=%x\n",Device,(Device<<15));
    fprintf(log_file,"Function,(Function<<12)=%x\n",Function,(Function<<12));
    fprintf(log_file,"Offset,(Offset)=%x\n",Offset,Offset);
#endif
    // aydin: how did he figure this out?
    byteaddress = (Bus<<20) | (Device<<15) | (Function<<12) | Offset;

    // why divide by 4?
    index = byteaddress / 4;
    return ( index );
}

void initializeMMConfigPtr()
{
    char filename[100];
    sprintf(filename,"/dev/mem");
	logger->debug("opening {}",filename);
    unsigned long mmconfig_base=0x80000000;		// DOUBLE-CHECK THIS ON NEW SYSTEMS!!!!!   grep MMCONFIG /proc/iomem | awk -F- '{print $1}'
    unsigned long mmconfig_size=0x10000000;

	int mem_fd = open(filename, O_RDWR);
	// fprintf(log_file,"   open command returns %d\n",mem_fd);
	if (mem_fd == -1) {
		logger->debug("ERROR {} when trying to open {}", strerror(errno), filename);
		exit(EXIT_FAILURE);
	}
    
	int map_prot = PROT_READ | PROT_WRITE;
	mmconfig_ptr = static_cast<unsigned int*>(mmap(NULL, mmconfig_size, map_prot, MAP_SHARED, mem_fd, mmconfig_base));
    if (mmconfig_ptr == MAP_FAILED) {
        logger->error("cannot mmap base of PCI configuration space from /dev/mem: address {:x}", mmconfig_base);
        exit(EXIT_FAILURE);
    } else {
        logger->info("Successful mmap of base of PCI configuration space from /dev/mem at address {:x}", mmconfig_base);
    }
    close(mem_fd);      // OK to close file after mmap() -- the mapping persists until unmap() or program exit


    	// New simple test that does not need to know the uncore bus numbers here...
	// Skylake bus 0, Function 5, offset 0 -- Sky Lake-E MM/Vt-d Configuration Registers
	//
	// simple test -- should return "20248086" on Skylake Xeon EP -- DID 0x2024, VID 0x8086
	uint32_t bus = 0x00;
	uint32_t device = 0x5;
	uint32_t function = 0x0;
	uint32_t offset = 0x0;
	uint32_t index = PCI_cfg_index(bus, device, function, offset);
    uint32_t value = mmconfig_ptr[index];

	if (value == 0x20248086) {
		logger->info("DEBUG: Well done! Bus {:x} device {:x} function {:x} offset {:x} returns expected value of {:x}", bus, device, function, offset, value);
	} else {
		logger->error("DEBUG: ERROR: Bus {:x} device {:x} function {:x} offset {:x} expected {:x}, found {:x}", bus, device, function, offset, 0x20248086, value);
		exit(EXIT_FAILURE);
	}
}

void readImcCounters()
{
    logger->info("Reading IMC counters.");
	
    uint32_t bus = IMC_BUS_Socket[0];
    for (uint32_t channel=0; channel<6; channel++) {
        uint32_t device = IMC_Device_Channel[channel];
        uint32_t function = IMC_Function_Channel[channel];
        for (uint32_t counter=0; counter<5; counter++) {
            uint32_t offset = IMC_PmonCtr_Offset[counter];
            uint32_t index = PCI_cfg_index(bus, device, function, offset);
            uint32_t low = mmconfig_ptr[index];
            uint32_t high = mmconfig_ptr[index+1];
            uint64_t count = ((uint64_t) high) << 32 | (uint64_t) low;
            
            logger->info("imc count: {}", count);
        }
    }
	
}




/// virt to phys translation

typedef struct {
    uint64_t pfn : 55;
    unsigned int soft_dirty : 1;
    unsigned int file_page : 1;
    unsigned int swapped : 1;
    unsigned int present : 1;
} PagemapEntry;

/* Parse the pagemap entry for the given virtual address.
 *
 * @param[out] entry      the parsed entry
 * @param[in]  pagemap_fd file descriptor to an open /proc/pid/pagemap file
 * @param[in]  vaddr      virtual address to get entry for
 * @return 0 for success, 1 for failure
 */
int pagemap_get_entry(PagemapEntry *entry, int pagemap_fd, uintptr_t vaddr)
{
    size_t nread;
    ssize_t ret;
    uint64_t data;
    uintptr_t vpn;

    vpn = vaddr / sysconf(_SC_PAGE_SIZE);
    nread = 0;
    while (nread < sizeof(data)) {
        ret = pread(pagemap_fd, ((uint8_t*)&data) + nread, sizeof(data) - nread,
                vpn * sizeof(data) + nread);
        nread += ret;
        if (ret <= 0) {
            return 1;
        }
    }
    entry->pfn = data & (((uint64_t)1 << 55) - 1);
    entry->soft_dirty = (data >> 55) & 1;
    entry->file_page = (data >> 61) & 1;
    entry->swapped = (data >> 62) & 1;
    entry->present = (data >> 63) & 1;
    return 0;
}

/* Convert the given virtual address to physical using /proc/PID/pagemap.
 *
 * @param[out] paddr physical address
 * @param[in]  pid   process to convert for
 * @param[in] vaddr virtual address to get entry for
 * @return 0 for success, 1 for failure
 */
int virt_to_phys_user(uintptr_t *paddr, pid_t pid, uintptr_t vaddr)
{
    char pagemap_file[BUFSIZ];
    int pagemap_fd;

    snprintf(pagemap_file, sizeof(pagemap_file), "/proc/%ju/pagemap", (uintmax_t)pid);
    pagemap_fd = open(pagemap_file, O_RDONLY);
    if (pagemap_fd < 0) {
        return 1;
    }
    PagemapEntry entry;
    if (pagemap_get_entry(&entry, pagemap_fd, vaddr)) {
        return 1;
    }
    close(pagemap_fd);
    *paddr = (entry.pfn * sysconf(_SC_PAGE_SIZE)) + (vaddr % sysconf(_SC_PAGE_SIZE));
    return 0;
}

int getIndex(long physical_address)
{
    /// 2^m = baseSequenceLength. m = 12 on SKX with 18 cores.
    /// Therefore "index" bits are 17:6. -> 12 bits in total represent index value.
    logger->debug("{} is called for physical address 0x{:x} (0b{:b}).", __PRETTY_FUNCTION__, physical_address, physical_address);

    physical_address = physical_address >> 6;
    logger->debug("shifted right to 6 bits: 0b{:b}", physical_address);

    int index_mask = 0xFFF;
    long index = physical_address & index_mask;
    logger->debug("index after mask is applied: 0b{:b}", index);

    logger->debug("physi -> 0b{:064b}", physical_address);
    logger->debug("index -> 0b{:064b}, 0x{:x}, {}", index, index, index);

    return index;
}

std::vector<int> readBaseSequence(const std::string& filename)
{
    std::vector<int> res;

    std::ifstream infile(filename);
    int a;
    while (infile >> a) {
        res.push_back(a);
    }

    // logger->debug("Base sequence: ");
    // for(auto num : res) {
    //     logger->debug(num);
    // }

    logger->debug("base sequence size: {}", res.size());

    return res;
}

// int main(int argc, char **argv)
// {
//     assertRoot();
//     pid_t pid;
//     uintptr_t vaddr, paddr = 0;

//     if (argc < 3) {
//         printf("Usage: %s pid vaddr\n", argv[0]);
//         return EXIT_FAILURE;
//     }
//     pid = strtoull(argv[1], NULL, 0);
//     vaddr = strtoull(argv[2], NULL, 0);
//     if (virt_to_phys_user(&paddr, pid, vaddr)) {
//         fprintf(stderr, "error: virt_to_phys_user\n");
//         return EXIT_FAILURE;
//     };
//     printf("0x%jx\n", (uintmax_t)paddr);
//     return EXIT_SUCCESS;
// }


















/// virt to phys translation

int main(int argc, char** argv)
{   
    std::signal(SIGHUP, SIG_IGN);
    setLogger("logs/cha_finding.txt");
    assertRoot(); /// assert root must come after setLogger() since it uses logger instance.
    logger->info("================================================ STARTING ================================================");
    
    pid_t pid = getpid();

    long long var = 10;
    uintptr_t physical_address = 0;

    uintptr_t virtual_address = reinterpret_cast<uintptr_t>(&var);

    if (virt_to_phys_user(&physical_address, pid, virtual_address)) {
        logger->error("error: virt_to_phys_user");
        return EXIT_FAILURE;
    };
    
    // printf("uintmax_t vaddr: 0x%jx\n", (uintmax_t)vaddr);
    // printf("uintptr_t vaddr: 0x%jx\n", (uintptr_t)vaddr);
    // printf("uintmax_t paddr: 0x%jx\n", (uintmax_t)paddr);
    // printf("uintptr_t paddr: 0x%jx\n", (uintptr_t)paddr);
    
    logger->info("virtual address 0x{:x} is mapped to physical address 0x{:x}.", virtual_address, physical_address);

    auto computed_perm = compute_perm(physical_address);

    auto physical_address_index = getIndex(physical_address);

    auto base_sequence_index = computed_perm ^ physical_address_index; /// XOR'ing.
    

    std::vector<int> base_sequence = readBaseSequence("BaseSequence_SKX_18-slice.txt");
    logger->debug("base_sequence_index: {}", base_sequence_index);

    assert(base_sequence_index < 4096 && "Base sequence must be lower than 4096!");
    int cha_by_hashing = base_sequence[base_sequence_index];
    logger->debug("CHA of virtual address 0x:{:x} is {}", virtual_address, cha_by_hashing);

    // void* cache_line = allocateCacheLine();
    // auto data = static_cast<long long*>(cache_line);
    int cha_by_perf_counters = findCHA(&var);

    logger->info("End result --> PerfCounters: {}, Hashing: {}", cha_by_perf_counters, cha_by_hashing);

    return 0;
    
    
    initializeMMConfigPtr(); /// will need this for IMC counters.




    return 0;
    // std::signal(SIGHUP, SIG_IGN);
    // setLogger("logs/traffic_measurement.txt");
    // logger->info("================================================ STARTING ================================================");

    // for(int i = 0; i < getCoreCount(); ++i) {
    //     readTrafficCountersWhileReadingData(i);
    //     sleep(1);
    // }

    // return 0;

    int istatus = pthread_spin_init(&g_lock, PTHREAD_PROCESS_PRIVATE);



    // if(argc != 3) {
    //     std::cerr << "./placer <CORE_NO> <CORE_NO>\n";
    //     return 0;
    // }

    std::signal(SIGHUP, SIG_IGN);


    
    // int main_thread_core_id = std::stoi(argv[1]);
    // int secondary_thread_core_id = std::stoi(argv[2]);;

    // stick_this_thread_to_core(main_thread_core_id);

    // testSameCacheLineCHAAssignment();


    const long long ping_pong_iteration_count = 1'000'000'000;
    logger->info("number of ping pong iterations : {} million.", ping_pong_iteration_count / 1'000'000);

    // std::vector<std::pair<int, int>> core_pairs{{0, 14}, {14, 8}, {8, 13}, {13, 3}, {10, 2}, {7,5}, {5,1}, {1,9}, {17, 15}, {10, 6}, {8,16}, {3, 11}, {11,5}};
    // std::vector<std::pair<int, int>> core_pairs{{14, 8}};
    std::vector<std::pair<int, int>> core_pairs{{0, 14}, {7, 5}, {17, 15}, {10, 6}, {13, 3}};
    
    for(const auto& core_pair : core_pairs){
        for(int i = 0; i < 5; ++i) {
            auto first_core  = core_pair.first;
            auto second_core = core_pair.second;

            void* cache_line = allocateCacheLine();
            auto data = static_cast<long long*>(cache_line);
            g_assigned_cha = findCHA(data);

            {
            ConfigParam config_param{LEFT_AD_READ, RIGHT_AD_READ, UP_AD_READ, DOWN_AD_READ, FILTER0_OFF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            logger->critical("#{} PingPongDuration: {} ns, core: {}, core: {}, cha: {}, iteration_count: {}", i+1, duration, first_core, second_core, g_assigned_cha, ping_pong_iteration_count);
            logger->flush();
            }

            {
            ConfigParam config_param{LEFT_BL_READ, RIGHT_BL_READ, UP_BL_READ, DOWN_BL_READ, FILTER0_OFF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            logger->critical("#{} PingPongDuration: {} ns, core: {}, core: {}, cha: {}, iteration_count: {}", i+1, duration, first_core, second_core, g_assigned_cha, ping_pong_iteration_count);
            logger->flush();
            }

            {
            ConfigParam config_param{LEFT_AK_READ, RIGHT_AK_READ, UP_AK_READ, DOWN_AK_READ, FILTER0_OFF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            logger->critical("#{} PingPongDuration: {} ns, core: {}, core: {}, cha: {}, iteration_count: {}", i+1, duration, first_core, second_core, g_assigned_cha, ping_pong_iteration_count);
            logger->flush();
            }

            {
            ConfigParam config_param{LEFT_IV_READ, RIGHT_IV_READ, UP_IV_READ, DOWN_IV_READ, FILTER0_OFF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            logger->critical("#{} PingPongDuration: {} ns, core: {}, core: {}, cha: {}, iteration_count: {}", i+1, duration, first_core, second_core, g_assigned_cha, ping_pong_iteration_count);
            logger->flush();
            }

            {
            ConfigParam config_param{HITME_LOOKUP_READ, HITME_LOOKUP_WRITE, LLC_DATA_READ_LOOKUP, LLC_WRITE_LOOKUP, FILTER0_ALL_COMBINED_LLC_SF, FILTER1_OFF};
            auto duration = runPingPongMicroBenchmark(data, ping_pong_iteration_count, first_core, second_core, config_param);
            logger->critical("#{} PingPongDuration: {} ns, core: {}, core: {}, cha: {}, iteration_count: {}", i+1, duration, first_core, second_core, g_assigned_cha, ping_pong_iteration_count);
            logger->flush();
            }
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
    pthread_spin_destroy(&g_lock);
}

/// returns assigned cha of allocated memory.
/// second param: which index of the first param (array) will be used. default is set 0 in declaration. (btw all indexes that map to same cache line will be mapped to same CHA.)
int findCHA(long long* data, int index)
{
    logger->debug("-------------------------------------------------------------- FINDCHA STARTED -------------------------------------------------------------------");
    logger->debug("index: {}", index);

    stick_this_thread_to_core(0); /// stick thread to any core.

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
        data[index] += 1;
        _mm_mfence();
        _mm_clflush(&data[index]);
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
    logger->info("assigned cha of address {:p}: {}", static_cast<void*>(&data[index]), assigned_cha);

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

static inline void incrementLoop(long long* data, long long iteration_count, int core_id)
{
    thread_started = true;

    stick_this_thread_to_core(core_id);	
    logger->info("secondary core {} modifying data {}, {} times.", core_id, static_cast<void*>(data), iteration_count);

    for(long long i = 0; i < iteration_count; ++i) {
        pthread_spin_lock(&g_lock);
        ++data[0];
        pthread_spin_unlock(&g_lock);
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

    // stick_this_thread_to_core(main_thread_core_id); /// actually we did this at startup, just to make sure, do it again!

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
    while(!thread_started)
    ;

    logger->info("------------------------------ PingPong started ------------------------------");
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    incrementLoop(data, iteration_count, main_thread_core_id);
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

        logger->info("CHA {} --> CTR0 weight: {}, CTR1 weight: {}, CTR2 weight: {}, CTR3 weight: {}", cha, var0, var1, var2, var3);

        val0s.insert({var0, cha});
        val1s.insert({var1, cha});
        val2s.insert({var2, cha});
        val3s.insert({var3, cha});

        // horizontal_vals.insert({left + right, cha});
        // vertical_vals.insert({up + down, cha});
        // total_vals.insert({left + right + up + down, cha});
    }

    logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR0], main_thread_core_id, secondary_thread_core_id, g_assigned_cha);
    for(const auto val0 : val0s) {
        logger->info("cha: {}, percentage: {}", val0.second, val0.first);
    }
    logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR1], main_thread_core_id, secondary_thread_core_id, g_assigned_cha);
    for(const auto val1 : val1s) {
        logger->info("cha: {}, percentage: {}", val1.second, val1.first);
    }
    logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR2], main_thread_core_id, secondary_thread_core_id, g_assigned_cha);
    for(const auto val2 : val2s) {
        logger->info("cha: {}, percentage: {}", val2.second, val2.first);
    }
    logger->info("---\nAnalyzing {}, cores: {}, {}; cha: {}:", descriptions[config_param.CTR3], main_thread_core_id, secondary_thread_core_id, g_assigned_cha);
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

    logger->debug("data last value: {}", data[0]);
    logger->debug("allocated virtual address: {:p}", static_cast<void*>(&data[0]));

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

void testSameCacheLineCHAAssignment()
{
    long long* cache_line = static_cast<long long*>(allocateCacheLine());
    g_assigned_cha = findCHA(cache_line, 0);
    g_assigned_cha = findCHA(cache_line, 1);
    g_assigned_cha = findCHA(cache_line, 2);
    g_assigned_cha = findCHA(cache_line, 3);
    g_assigned_cha = findCHA(cache_line, 4);
    g_assigned_cha = findCHA(cache_line, 5);
    g_assigned_cha = findCHA(cache_line, 6);
    g_assigned_cha = findCHA(cache_line, 7);
}
