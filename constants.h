#pragma once

/// general
static constexpr int CACHE_LINE_SIZE  = 64;
static constexpr int NUM_SOCKETS      = 1;
static constexpr int NUM_CHA_BOXES    = 18;

/// base
static constexpr long CHA_MSR_PMON_CTRL_BASE = 0x0E01L;
static constexpr long CHA_MSR_PMON_CTR_BASE  = 0x0E08L;

/// mesh traffic
static constexpr unsigned int LEFT_READ  = 0x004003AB; /// horizontal_bl_ring
static constexpr unsigned int RIGHT_READ = 0x00400CAB; /// horizontal_bl_ring
static constexpr unsigned int UP_READ    = 0x004003AA; /// vertical_bl_ring
static constexpr unsigned int DOWN_READ  = 0x00400CAA; /// vertical_bl_ring

///// cache

// llc lookups

// Definition: Counts the number of times the LLC was accessed - this includes code,
// data, prefetches and hints coming from L2. This has numerous filters available. Note
// the non-standard filtering equation. This event will count requests that lookup the
// cache multiple times with multiple increments. One must **************ALWAYS************** set umask bit 0
// AND SELECT A STATE OR STATES TO MATCH. --> Otherwise, the event will count nothing. <-- CHA-
// Filter0[24:21,17] bits correspond to [FMESI] state.
// • NOTE: ***Bit 0 of the umask must ***ALWAYS*** be set for this event***. This allows us to match
// against a given state (or states) as programmed in the Cn_MSR_PMON_BOX_FIL-
// TER0.state field bitmask. 0 = I (miss), 4 = S, 5 = E, 6 = M, 7 = F. For example, if
// you wanted to monitor F and S hits, you could set 00001001b in the 8-bit state field.
// To monitor any lookup, set the field to 0x1F. Extra note - it may be a little confusing
// for customers of earlier products. With the CBo and HA functionality combined, it's
// possible to also measure Snoop Filter lookups with bits 1-3 of the FILTER0.state field



/// LLC_LOOKUP may be filtered by the cacheline state (using CHA filter registers).
static constexpr unsigned int LLC_ANY_LOOKUP          = 0x00401134;
static constexpr unsigned int LLC_LOCAL_LOOKUP        = 0x00403134;
static constexpr unsigned int LLC_REMOTE_LOOKUP       = 0x00409134;
static constexpr unsigned int LLC_DATA_READ_LOOKUP    = 0x00400334;
static constexpr unsigned int LLC_WRITE_LOOKUP        = 0x00400534;
static constexpr unsigned int LLC_REMOTE_SNOOP_LOOKUP = 0x00400934;

static constexpr unsigned int DIR_LOOKUP_SNP_NOSNP = 0x00400353; /// umask is SNP | NO_SNP
static constexpr unsigned int DIR_LOOKUP_SNP = 0x00400153; /// umask is SNP
static constexpr unsigned int DIR_LOOKUP_NOSNP = 0x00400253; /// umask is NO_SNP

static constexpr unsigned int XSNP_RESP_EVICT_RSP_HITFSE = 0x00408132;
static constexpr unsigned int SF_EVICTION_M_STATE        = 0x0040013d;
static constexpr unsigned int SF_EVICTION_E_STATE        = 0x0040023d;
static constexpr unsigned int SF_EVICTION_S_STATE        = 0x0040043d;

/// filter
static constexpr unsigned int FILTER0_OFF = 0;
static constexpr unsigned int FILTER0_ALL_LLC = (1 << 24) | (1 << 23) | (1 << 22) | (1 << 21) | (1 << 17); /// for filtering only and all LLC events (FMESI).
// static constexpr unsigned int FILTER0_ALL_SF = 0x0000003B; /// for filtering only LLC events.
static constexpr unsigned int FILTER1_OFF = 0x0000003B; /// 3B essentially turns off this filter.