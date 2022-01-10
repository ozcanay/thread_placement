#pragma once

#include <cstdint> // for uintptr_t
#include <unistd.h> /// for pid_t
#include <string>
#include <vector>

struct PagemapEntry;

void ignoreHangupSignal();
void assertRoot();
void* allocateCacheLine();
void* allocateChunk(int cache_line_count);

int virt_to_phys_user(uintptr_t *paddr, pid_t pid, uintptr_t vaddr);
uintptr_t virt_to_phys_user2(uintptr_t vaddr);

int pagemap_get_entry(PagemapEntry *entry, int pagemap_fd, uintptr_t vaddr);
int getIndex(long physical_address);
std::vector<int> readBaseSequence(const std::string& filename);