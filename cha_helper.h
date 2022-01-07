#pragma once

int compute_perm(long physical_address);
/// returns assigned cha of allocated memory. USES PERFORMANCE COUNTERS!
/// second param: which index of the first param (array) will be used. default is set 0 in declaration. (btw all indexes that map to same cache line will be mapped to same CHA.)
int findCHA(long long* data, int index = 0);
/// it is important to get the pointer by reference so that we do not copy it here! --> I AM NOT SURE ABOUT THIS, POINTER STILL POINTS TO THE SAME MEMORY LOCATION.
int findCHAByHashing(long long*& val);
void runDifferentChaFindMethodsToCompare();