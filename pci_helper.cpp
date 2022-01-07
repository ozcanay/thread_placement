#include "pci_helper.h"
#include "logger_helper.h"

#include <sys/mman.h> // mmap(), PROT_READ, PROT_WRITE, MAP_FAILED
#include <unistd.h> // close()
#include <fcntl.h> // O_RDWR

extern unsigned int *mmconfig_ptr;

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
