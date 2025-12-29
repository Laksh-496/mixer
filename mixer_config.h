#ifndef __MIXER_CONFIG_H__
#define __MIXER_CONFIG_H__

#include "gpi/platform_spec.h"
#include "gpi/tools.h"

// --- D-CUBE 50-NODE CONFIGURATION ---

// Hardcoded IDs for 50 nodes (Assumes D-Cube IDs 1 through 50)
static const uint8_t nodes[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50
};

// Each node transmits its own ID as a message in the generation
static const uint8_t payload_distribution[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50
};

/*****************************************************************************/

// Scaled for 50 nodes. Rule of thumb: Slots >= 2 * Nodes for high reliability
#define MX_ROUND_LENGTH         100 
#define MX_NUM_NODES            NUM_ELEMENTS(nodes)
#define MX_GENERATION_SIZE      NUM_ELEMENTS(payload_distribution)
#define MX_PAYLOAD_SIZE         16
#define MX_INITIATOR_ID         1   // Node 1 is the initiator

// D-Cube standard is usually IEEE 802.15.4 (Mode 1)
#define MX_PHY_MODE             1
#define MX_TX_PWR_DBM           8

// Use Smart Shutdown Mode 3 (All nodes full rank) for large-scale stability
#define MX_SMART_SHUTDOWN       1
#define MX_SMART_SHUTDOWN_MODE  3

// Keep statistics on for D-Cube logs
#define MX_VERBOSE_STATISTICS   1
#define MX_VERBOSE_PACKETS      0

#endif // __MIXER_CONFIG_H__
