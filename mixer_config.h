#ifndef __MIXER_CONFIG_H__
#define __MIXER_CONFIG_H__

// mixer configuration file
// Adapted for auto node ID assignment on FIT IoT testbed

#include "gpi/platform_spec.h"		// GPI_ARCH_IS_...
#include "gpi/tools.h"				// NUM_ELEMENTS()

/*****************************************************************************/
/* AUTO NODE ID ASSIGNMENT CONFIGURATION *************************************/
/*****************************************************************************/

// Enable automatic node discovery and ID assignment
#define MX_AUTO_NODE_ASSIGNMENT		1

// Discovery phase duration in milliseconds (time to discover all nodes)
#define MX_DISCOVERY_DURATION_MS	3000

// Discovery beacon interval in milliseconds
#define MX_DISCOVERY_BEACON_INTERVAL_MS	200

// Maximum number of nodes supported in the network
#define MX_MAX_NODES				10

// Coordinator selection method:
// 0 = Lowest device ID becomes node 1 (most deterministic)
// 1 = Highest device ID becomes node 1
#define MX_COORDINATOR_SELECTION	0

/*****************************************************************************/
/* BASIC SETTINGS (will be populated dynamically) ****************************/
/*****************************************************************************/

// These will be filled during discovery phase
// Default values for 2 nodes (minimum configuration)
#define MX_ROUND_LENGTH			50 // in slots

// Will be set dynamically after discovery
extern uint8_t mx_num_nodes;
extern uint8_t mx_node_id;
extern uint8_t mx_generation_size;
extern uint8_t mx_initiator_id;

// Macros for runtime values
#define MX_NUM_NODES			mx_num_nodes
#define MX_GENERATION_SIZE		mx_generation_size
#define MX_INITIATOR_ID			mx_initiator_id

#define MX_PAYLOAD_SIZE			16

// choose a slot length according to your settings
// NOTE: Measurement unit is hybrid ticks.
#define MX_SLOT_LENGTH			GPI_TICK_US_TO_HYBRID2(2000)

// Possible values (Gpi_Radio_Mode):
//		IEEE_802_15_4	= 1
//		BLE_1M			= 2
//		BLE_2M			= 3
//		BLE_125k		= 4
//		BLE_500k		= 5
#define MX_PHY_MODE				1

// Values mentioned in the manual (nRF52840_PS_v1.1):
// +8dBm,  +7dBm,  +6dBm,  +5dBm,  +4dBm,  +3dBm, + 2dBm,
//  0dBm,  -4dBm,  -8dBm, -12dBm, -16dBm, -20dBm, -40dBm
#define MX_TX_PWR_DBM			8

/*****************************************************************************/
/* SPECIAL SETTINGS **********************************************************/
/*****************************************************************************/

#define MX_WEAK_ZEROS			0
#define WEAK_RELEASE_SLOT		1

#define MX_WARMSTART			0
#define WARMSTART_HISTORY		1

#define MX_REQUEST				1
#define MX_REQUEST_HEURISTIC	2

#define MX_SMART_SHUTDOWN		1
// 0	no smart shutdown
// 1	no unfinished neighbor, without full-rank map(s)
// 2	no unfinished neighbor
// 3	all nodes full rank
// 4	all nodes full rank, all neighbors ACKed knowledge of this fact
// 5	all nodes full rank, all nodes ACKed knowledge of this fact
#define MX_SMART_SHUTDOWN_MODE	2

// turn verbose log messages on or off
#define MX_VERBOSE_STATISTICS	1
#define MX_VERBOSE_PACKETS		0
#define MX_VERBOSE_PROFILE		0

/*****************************************************************************/
/* DISCOVERY PROTOCOL PACKET FORMAT ******************************************/
/*****************************************************************************/

// Discovery beacon structure
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];      // 0xAA, 0x55 - identification
    uint64_t device_id;     // nRF52840 unique device ID
    uint8_t  num_seen;      // Number of nodes this node has seen
    int8_t   rssi_hint;     // RSSI for coordinator selection (if used)
} discovery_beacon_t;

#define DISCOVERY_MAGIC_0	0xAA
#define DISCOVERY_MAGIC_1	0x55

#endif // __MIXER_CONFIG_H__