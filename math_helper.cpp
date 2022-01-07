#include "math_helper.h"

#include <random>

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