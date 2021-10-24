#include "thread_helper.h"
#include "logger_helper.h"

#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>

void stick_this_thread_to_core(int core_id) {
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (core_id < 0 || core_id >= num_cores) {
        logger->error("Core {} could not be bound to this thread.", core_id);
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();

    int res = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);

    if(res == 0) {
        logger->info("Thread bound to core {} successfully.", core_id);
    } else {
        logger->error("Error in binding this thread to core {}.", core_id);
    }
}