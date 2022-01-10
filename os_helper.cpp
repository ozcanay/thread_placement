#include "os_helper.h"

#include <fstream>

#include <cstdio> // for BUFSIZ
#include <fcntl.h> // for O_RDONLY, open()
#include "logger_helper.h"
#include "constants.h"

#include <iostream>
#include <csignal>

struct PagemapEntry
{
    uint64_t pfn : 55;
    unsigned int soft_dirty : 1;
    unsigned int file_page : 1;
    unsigned int swapped : 1;
    unsigned int present : 1;
};

void ignoreHangupSignal()
{
    std::signal(SIGHUP, SIG_IGN);
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

void* allocateCacheLine()
{
    void *v_data = nullptr;
    /// allocate heap of size equal to just 1 cache line.
    posix_memalign(&v_data, CACHE_LINE_SIZE, CACHE_LINE_SIZE); /// also try alignas 64 here from c++.
    logger->trace("allocated virtual data address: {:p}", v_data);

    return v_data;
}

void* allocateChunk(int cache_line_count)
{
    void *v_data = nullptr;
    posix_memalign(&v_data, CACHE_LINE_SIZE, cache_line_count * CACHE_LINE_SIZE); /// also try alignas 64 here from c++.
    // logger->trace("allocated virtual data address: {:p}", v_data);

    return v_data;
}

/// I am not using this at the moment, had come across this on web: https://stackoverflow.com/a/28987409/4645121
uintptr_t virt_to_phys_user2(uintptr_t vaddr) {
    FILE *pagemap;
    intptr_t paddr = 0;
    int offset = (vaddr / sysconf(_SC_PAGESIZE)) * sizeof(uint64_t);
    uint64_t e;

    // https://www.kernel.org/doc/Documentation/vm/pagemap.txt
    if ((pagemap = fopen("/proc/self/pagemap", "r"))) {
        if (lseek(fileno(pagemap), offset, SEEK_SET) == offset) {
            if (fread(&e, sizeof(uint64_t), 1, pagemap)) {
                if (e & (1ULL << 63)) { // page present ?
                    paddr = e & ((1ULL << 54) - 1); // pfn mask
                    paddr = paddr * sysconf(_SC_PAGESIZE);
                    // add offset within page
                    paddr = paddr | (vaddr & (sysconf(_SC_PAGESIZE) - 1));
                }   
            }   
        }   
        fclose(pagemap);
    }   

    return paddr;
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
        ret = pread(pagemap_fd, ((uint8_t*)&data) + nread, sizeof(data) - nread, vpn * sizeof(data) + nread);
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