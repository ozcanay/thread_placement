#include "imc_helper.h"
#include "logger_helper.h"
#include "pci_helper.h"
#include "intel_helper.h"
#include "thread_helper.h"
#include "constants.h"

#include <unistd.h> // sleep()

extern unsigned int *mmconfig_ptr;

/// page 26 uncore perf monitoring manual.
// int IMC_BUS_Socket[2] = {0x3a, 0xae}; /// where did we get this address from? --> lspci -d 8086:2042
int IMC_BUS_Socket[1] = {0x64};
int IMC_Device_Channel[6] = {0x0a, 0x0a, 0x0b, 0x0c, 0x0c, 0x0d}; /// device addresses in each channel.
int IMC_Function_Channel[6] = {0x2, 0x6, 0x2, 0x2, 0x6, 0x2}; /// function addresses in each channel.
int IMC_PmonCtl_Offset[5] = {0xd8, 0xdc, 0xe0, 0xe4, 0xf0}; /// 5 counters in total, last one DCLK
int IMC_PmonCtr_Offset[5] = {0xa0, 0xa8, 0xb0, 0xb8, 0xd0}; /// 5 counters in total, last one DCLK

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
            
            logger->debug("Reading bus: 0x{:x}, channel: {}, device: 0x{:x}, function: 0x{:x}, counter: {},"
                " offset: 0x{:x}, index: 0x{:x}, low: 0x{:x}, high: 0x{:x} --> count: {} (0x{:x})", 
                bus, channel, device, function, counter, offset, index, low, high, count, count);
        }
    }
}

void configureImcCounters(const std::string& config_file)
{
    logger->info("Programming IMC counters.");

	char filename[100];
    FILE *input_file;

	sprintf(filename,config_file.c_str());
	input_file = fopen(filename,"r");
	if (input_file == 0) {
        logger->error("Error trying to open file!");
		exit(-1);
	}

    char description[100];
    uint32_t bus, device, function, offset, ctl_offset, ctr_offset, value, index;
	uint32_t dummy32u;
	uint32_t socket, imc, channel, subchannel, counter;
    int rc;

	int i = 0;
	while (i < 100) {
		rc = fscanf(input_file,"%d %d %d %d %x %s",&socket,&imc,&subchannel,&counter,&value,&description);
		if (rc == EOF) break;
		i++;
		channel = 3*imc + subchannel;				// PCI device/function is indexed by channel here (0-5)
		bus = IMC_BUS_Socket[socket];
		device = IMC_Device_Channel[channel];
		function = IMC_Function_Channel[channel];
		offset = IMC_PmonCtl_Offset[counter];
		// fprintf(log_file,"DEBUG: translated bus/device/function/offset values %#x %#x %#x %#x\n",bus,device,function,offset);
		index = PCI_cfg_index(bus, device, function, offset);
		mmconfig_ptr[index] = value;

        logger->debug("i: {}, Writing socket: {}, bus: 0x{:x}, channel: {}, device: 0x{:x}, function: 0x{:x}, offset: 0x{:x}, index: {} --> value: {:08x}", i, socket, bus, channel, device, function, offset, index, value);
	}

	fclose(input_file);
}

void runImcBenchmark(bool flush)
{

    initializeMMConfigPtr(); /// will need this for IMC counters.
    configureImcCounters("imc_perfevtsel.input");

    long long g_x = 0;
    
    long long iteration = 100'000'000;

    for(int ii = 0; ii < 18; ++ii) {
        long long total_read_before = 0;
        long long total_read_after = 0;
        long long total_write_before = 0;
        long long total_write_after = 0;

        stick_this_thread_to_core(ii);
        
        // logger->info("Reading IMC counters.");
        for (uint32_t socket=0; socket<1; socket++) {
            uint32_t bus = IMC_BUS_Socket[socket];
            for (uint32_t channel=0; channel<NUM_IMC_CHANNELS; channel++) {
                uint32_t device = IMC_Device_Channel[channel];
                uint32_t function = IMC_Function_Channel[channel];

                for (uint32_t counter=0; counter<NUM_IMC_COUNTERS; counter++) {
                    uint32_t offset = IMC_PmonCtr_Offset[counter];
                    uint32_t index = PCI_cfg_index(bus, device, function, offset);
                    uint32_t low = mmconfig_ptr[index];
                    uint32_t high = mmconfig_ptr[index+1];
                    uint64_t count = ((uint64_t) high) << 32 | (uint64_t) low;

                    // logger->debug("Reading socket: {}, bus: {:x}, channel: {}, device: {:x}, function: {:x}, counter: {},"
                    // " offset: {:x}, index: {:x}, low: {:x}, high: {:x} --> count: {} (0x{:x})", 
                    // socket, bus, channel, device, function, counter, offset, index, low, high, count, count);

                    if(counter == 0) {
                        total_read_before += count;
                    } else if(counter == 1) {
                        total_write_before += count;
                    }
                }
            }
        }

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        
        if(flush) {
            for(long long i = 0; i < iteration; ++i) {
                g_x += 1;
                _mm_mfence();
                _mm_clflush(&g_x);
                _mm_mfence();
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        unsigned long long elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();


        for (uint32_t socket=0; socket<1; socket++) {
            uint32_t bus = IMC_BUS_Socket[socket];
            for (uint32_t channel=0; channel<NUM_IMC_CHANNELS; channel++) {
                uint32_t device = IMC_Device_Channel[channel];
                uint32_t function = IMC_Function_Channel[channel];

                for (uint32_t counter=0; counter<NUM_IMC_COUNTERS; counter++) {
                    uint32_t offset = IMC_PmonCtr_Offset[counter];
                    uint32_t index = PCI_cfg_index(bus, device, function, offset);
                    uint32_t low = mmconfig_ptr[index];
                    uint32_t high = mmconfig_ptr[index+1];
                    uint64_t count = ((uint64_t) high) << 32 | (uint64_t) low;

                    // logger->debug("Reading socket: {}, bus: {:x}, channel: {}, device: {:x}, function: {:x}, counter: {},"
                    // " offset: {:x}, index: {:x}, low: {:x}, high: {:x} --> count: {} (0x{:x})", 
                    // socket, bus, channel, device, function, counter, offset, index, low, high, count, count);

                    if(counter == 0) {
                        total_read_after += count;
                    } else if(counter == 1) {
                        total_write_after += count;
                    }
                }
            }
        }

        logger->info("read diff: {} million", (total_read_after - total_read_before) / 1'000'000.0);
        logger->info("write diff: {} million", (total_write_after - total_write_before) / 1'000'000.0);
        logger->info("iteration: {}", iteration);
        logger->info("core: {}, elapsed_time: {}ms", ii, elapsed_time);
        logger->info("-------------------------------------");
        sleep(3);
    }
}