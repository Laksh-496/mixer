/***************************************************************************************************
 ***************************************************************************************************
 *
 *	Copyright (c) 2019, Networked Embedded Systems Lab, TU Dresden
 *	All rights reserved.
 *
 *	Modified for automatic node ID assignment in FIT IoT testbed
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are met:
 *		* Redistributions of source code must retain the above copyright
 *		  notice, this list of conditions and the following disclaimer.
 *		* Redistributions in binary form must reproduce the above copyright
 *		  notice, this list of conditions and the following disclaimer in the
 *		  documentation and/or other materials provided with the distribution.
 *		* Neither the name of the NES Lab or TU Dresden nor the
 *		  names of its contributors may be used to endorse or promote products
 *		  derived from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 *	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************//**
 *
 *	@file					main.c
 *
 *	@brief					main entry point with auto node ID assignment
 *
 *	@version				$Id$
 *	@date					Modified for auto assignment
 *
 *	@author					Fabian Mager (original), Modified for auto assignment
 *
 ***************************************************************************************************

 	@details

	Implements automatic node discovery and ID assignment for FIT IoT testbed deployment.
	Nodes discover each other via beacon exchange, then deterministically assign IDs based
	on hardware unique device IDs. The node with the lowest (or highest) device ID becomes
	the coordinator (node 1/initiator).

 **************************************************************************************************/
//***** Trace Settings *****************************************************************************

#include "gpi/trace.h"

// message groups for TRACE messages (used in GPI_TRACE_MSG() calls)
#define TRACE_INFO		GPI_TRACE_MSG_TYPE_INFO

// select active message groups
#ifndef GPI_TRACE_BASE_SELECTION
	#define GPI_TRACE_BASE_SELECTION	GPI_TRACE_LOG_STANDARD | GPI_TRACE_LOG_PROGRAM_FLOW
#endif
GPI_TRACE_CONFIG(main, GPI_TRACE_BASE_SELECTION);

//**************************************************************************************************
//***** Includes ***********************************************************************************

#include "mixer/mixer.h"

#include "gpi/tools.h"
#include "gpi/platform.h"
#include "gpi/interrupts.h"
#include "gpi/clocks.h"
#include "gpi/olf.h"
#include GPI_PLATFORM_PATH(radio.h)

#include <nrf.h>

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

//**************************************************************************************************
//***** Local Defines and Consts *******************************************************************

#define PRINT_HEADER()		printf("# ID:%u ", TOS_NODE_ID)

//**************************************************************************************************
//***** Local Typedefs and Class Declarations ******************************************************

// Structure to hold information about discovered nodes
typedef struct {
    uint64_t device_id;     // Unique nRF52840 device ID
    int8_t   rssi;          // Signal strength (if needed for selection)
    uint32_t last_seen_ms;  // Timestamp of last beacon
    uint8_t  active;        // 1 if node is active
} node_info_t;

// Discovery state
typedef struct {
    uint64_t    my_device_id;           // This node's device ID
    node_info_t nodes[MX_MAX_NODES];    // Discovered nodes
    uint8_t     num_discovered;         // Count of discovered nodes
    uint8_t     my_logical_id;          // My assigned logical ID (0-based)
    uint8_t     is_coordinator;         // 1 if I am the coordinator
    uint8_t     discovery_complete;     // Discovery phase finished
} discovery_state_t;

//**************************************************************************************************
//***** Forward Declarations ***********************************************************************

static void run_discovery_phase(void);
static void assign_node_ids(void);
static uint64_t get_device_id(void);
static void send_discovery_beacon(void);
static void discovery_rx_callback(uint8_t *payload, uint8_t length, int8_t rssi);

//**************************************************************************************************
//***** Local (Static) Variables *******************************************************************

static uint8_t		node_id;
static uint32_t		round;
static uint32_t		msgs_decoded;
static uint32_t		msgs_not_decoded;
static uint32_t		msgs_weak;
static uint32_t		msgs_wrong;

// Discovery state
static discovery_state_t discovery_state;

// Dynamic configuration (extern declarations from mixer_config.h)
uint8_t mx_num_nodes = 2;          // Default minimum
uint8_t mx_node_id = 0;
uint8_t mx_generation_size = 2;    // Will be updated based on discovered nodes
uint8_t mx_initiator_id = 1;       // First node in payload_distribution

// Dynamic payload distribution array
static uint8_t payload_distribution[MX_MAX_NODES * 4];  // Max messages
static uint8_t nodes[MX_MAX_NODES];                      // Physical node IDs

// Flag for discovery beacon reception
static volatile uint8_t discovery_rx_flag = 0;
static uint8_t discovery_rx_buffer[32];
static uint8_t discovery_rx_length = 0;
static int8_t discovery_rx_rssi = 0;

//**************************************************************************************************
//***** Global Variables ***************************************************************************

// TOS_NODE_ID - will be set during discovery to match assigned logical ID
uint16_t __attribute__((section(".data")))	TOS_NODE_ID = 0;

//**************************************************************************************************
//***** Discovery Functions ************************************************************************

// Get nRF52840 unique device ID from FICR
static uint64_t get_device_id(void)
{
    uint64_t device_id;
    
    // Read 64-bit unique device ID from Factory Information Configuration Registers
    device_id = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
    
    return device_id;
}

//**************************************************************************************************

// Send discovery beacon
static void send_discovery_beacon(void)
{
    discovery_beacon_t beacon;
    
    beacon.magic[0] = DISCOVERY_MAGIC_0;
    beacon.magic[1] = DISCOVERY_MAGIC_1;
    beacon.device_id = discovery_state.my_device_id;
    beacon.num_seen = discovery_state.num_discovered;
    beacon.rssi_hint = 0;  // Can be used for RSSI-based selection
    
    // Send beacon using radio
    // Use a simple transmission without Mixer protocol
    gpi_radio_set_channel(39);  // BLE channel for discovery
    
    // Transmit the beacon
    uint8_t *payload = (uint8_t *)&beacon;
    
    // Simple blocking transmission for discovery
    // In production, this would use interrupt-driven transmission
    NRF_RADIO->PACKETPTR = (uint32_t)payload;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    
    // Wait for transmission to complete
    while (!NRF_RADIO->EVENTS_END);
    NRF_RADIO->EVENTS_END = 0;
}

//**************************************************************************************************

// Process received discovery beacon
static void process_discovery_beacon(uint8_t *payload, uint8_t length, int8_t rssi)
{
    discovery_beacon_t *beacon;
    
    // Validate beacon
    if (length < sizeof(discovery_beacon_t))
        return;
    
    beacon = (discovery_beacon_t *)payload;
    
    if (beacon->magic[0] != DISCOVERY_MAGIC_0 || beacon->magic[1] != DISCOVERY_MAGIC_1)
        return;
    
    // Don't add ourselves
    if (beacon->device_id == discovery_state.my_device_id)
        return;
    
    // Check if already discovered
    for (uint8_t i = 0; i < discovery_state.num_discovered; i++)
    {
        if (discovery_state.nodes[i].device_id == beacon->device_id)
        {
            // Update existing entry
            discovery_state.nodes[i].rssi = rssi;
            discovery_state.nodes[i].last_seen_ms = gpi_tick_hybrid_to_us(gpi_tick_hybrid()) / 1000;
            return;
        }
    }
    
    // Add new node
    if (discovery_state.num_discovered < MX_MAX_NODES)
    {
        uint8_t idx = discovery_state.num_discovered;
        discovery_state.nodes[idx].device_id = beacon->device_id;
        discovery_state.nodes[idx].rssi = rssi;
        discovery_state.nodes[idx].last_seen_ms = gpi_tick_hybrid_to_us(gpi_tick_hybrid()) / 1000;
        discovery_state.nodes[idx].active = 1;
        discovery_state.num_discovered++;
        
        printf("Discovered node: DevID=0x%08X%08X, RSSI=%d dBm, Total=%u\n",
               (uint32_t)(beacon->device_id >> 32),
               (uint32_t)(beacon->device_id & 0xFFFFFFFF),
               rssi,
               discovery_state.num_discovered);
    }
}

//**************************************************************************************************

// Comparison function for sorting by device ID (ascending)
static int compare_device_ids_asc(const void *a, const void *b)
{
    const node_info_t *na = (const node_info_t *)a;
    const node_info_t *nb = (const node_info_t *)b;
    
    if (na->device_id < nb->device_id) return -1;
    if (na->device_id > nb->device_id) return 1;
    return 0;
}

//**************************************************************************************************

// Comparison function for sorting by device ID (descending)
static int compare_device_ids_desc(const void *a, const void *b)
{
    return -compare_device_ids_asc(a, b);
}

//**************************************************************************************************

// Assign node IDs based on discovery results
static void assign_node_ids(void)
{
    node_info_t all_nodes[MX_MAX_NODES];
    uint8_t total_nodes = 0;
    
    // Copy discovered nodes
    memcpy(all_nodes, discovery_state.nodes, discovery_state.num_discovered * sizeof(node_info_t));
    total_nodes = discovery_state.num_discovered;
    
    // Add this node to the list
    all_nodes[total_nodes].device_id = discovery_state.my_device_id;
    all_nodes[total_nodes].rssi = -30;  // Assume good signal to self
    all_nodes[total_nodes].active = 1;
    total_nodes++;
    
    // Sort nodes based on selection method
    if (MX_COORDINATOR_SELECTION == 0)
    {
        // Lowest device ID becomes coordinator
        qsort(all_nodes, total_nodes, sizeof(node_info_t), compare_device_ids_asc);
    }
    else
    {
        // Highest device ID becomes coordinator
        qsort(all_nodes, total_nodes, sizeof(node_info_t), compare_device_ids_desc);
    }
    
    // Find our position and assign logical ID (0-based for internal use)
    for (uint8_t i = 0; i < total_nodes; i++)
    {
        if (all_nodes[i].device_id == discovery_state.my_device_id)
        {
            discovery_state.my_logical_id = i;  // 0-based
            discovery_state.is_coordinator = (i == 0);
            break;
        }
    }
    
    // Update global configuration
    mx_num_nodes = total_nodes;
    mx_node_id = discovery_state.my_logical_id;
    
    // Build nodes array (physical IDs = logical IDs + 1 for compatibility)
    for (uint8_t i = 0; i < total_nodes; i++)
    {
        nodes[i] = i + 1;
    }
    
    // Build payload distribution (each node sends messages equal to its ID)
    // Simple distribution: each node sends 1-2 messages
    mx_generation_size = 0;
    for (uint8_t i = 0; i < total_nodes; i++)
    {
        uint8_t msgs_per_node = 2;  // Each node sends 2 messages
        for (uint8_t j = 0; j < msgs_per_node; j++)
        {
            payload_distribution[mx_generation_size++] = i + 1;
        }
    }
    
    // Initiator is always the first node (coordinator)
    mx_initiator_id = payload_distribution[0];
    
    // Set TOS_NODE_ID to match our logical position (1-based for compatibility)
    TOS_NODE_ID = discovery_state.my_logical_id + 1;
    
    printf("\n");
    printf("========================================\n");
    printf("   Node ID Assignment Complete\n");
    printf("========================================\n");
    printf("My Device ID:    0x%08X%08X\n",
           (uint32_t)(discovery_state.my_device_id >> 32),
           (uint32_t)(discovery_state.my_device_id & 0xFFFFFFFF));
    printf("Logical ID:      %u (0-based)\n", discovery_state.my_logical_id);
    printf("TOS_NODE_ID:     %u\n", TOS_NODE_ID);
    printf("Role:            %s\n", discovery_state.is_coordinator ? "COORDINATOR (Initiator)" : "PARTICIPANT");
    printf("Total Nodes:     %u\n", total_nodes);
    printf("Generation Size: %u\n", mx_generation_size);
    printf("Initiator ID:    %u\n", mx_initiator_id);
    printf("========================================\n");
    printf("\n");
    
    // Print all nodes in order
    printf("Network Topology (sorted by device ID):\n");
    for (uint8_t i = 0; i < total_nodes; i++)
    {
        printf("  Node %u: DevID=0x%08X%08X %s\n",
               i + 1,
               (uint32_t)(all_nodes[i].device_id >> 32),
               (uint32_t)(all_nodes[i].device_id & 0xFFFFFFFF),
               (all_nodes[i].device_id == discovery_state.my_device_id) ? "<-- ME" : "");
    }
    printf("\n");
}

//**************************************************************************************************

// Run discovery phase
static void run_discovery_phase(void)
{
    uint32_t discovery_start_ms, current_ms, last_beacon_ms = 0;
    
    printf("\n");
    printf("========================================\n");
    printf("   Starting Node Discovery Phase\n");
    printf("========================================\n");
    printf("Duration:        %u ms\n", MX_DISCOVERY_DURATION_MS);
    printf("Beacon Interval: %u ms\n", MX_DISCOVERY_BEACON_INTERVAL_MS);
    printf("My Device ID:    0x%08X%08X\n",
           (uint32_t)(discovery_state.my_device_id >> 32),
           (uint32_t)(discovery_state.my_device_id & 0xFFFFFFFF));
    printf("========================================\n");
    printf("\n");
    
    // Initialize discovery state
    memset(&discovery_state, 0, sizeof(discovery_state_t));
    discovery_state.my_device_id = get_device_id();
    
    // Get start time
    discovery_start_ms = gpi_tick_hybrid_to_us(gpi_tick_hybrid()) / 1000;
    
    // Setup radio for discovery (simpler mode, no Mixer)
    gpi_radio_init(BLE_1M);
    gpi_radio_set_channel(39);  // BLE advertising channel
    
    // Discovery loop
    while (1)
    {
        current_ms = gpi_tick_hybrid_to_us(gpi_tick_hybrid()) / 1000;
        
        // Check if discovery phase is complete
        if ((current_ms - discovery_start_ms) >= MX_DISCOVERY_DURATION_MS)
            break;
        
        // Send beacon periodically
        if ((current_ms - last_beacon_ms) >= MX_DISCOVERY_BEACON_INTERVAL_MS)
        {
            send_discovery_beacon();
            last_beacon_ms = current_ms;
        }
        
        // Listen for beacons (simplified polling for this example)
        // In production, use interrupt-driven reception
        // Check if there's a pending received packet
        if (discovery_rx_flag)
        {
            discovery_rx_flag = 0;
            process_discovery_beacon(discovery_rx_buffer, discovery_rx_length, discovery_rx_rssi);
        }
        
        // Small delay to prevent busy-waiting
        gpi_milli_sleep(10);
    }
    
    discovery_state.discovery_complete = 1;
    
    printf("Discovery phase complete. Discovered %u other node(s).\n\n", 
           discovery_state.num_discovered);
    
    // Assign IDs based on discovery results
    assign_node_ids();
}

//**************************************************************************************************
//***** Original Mixer Functions *******************************************************************

// Print results of a Mixer round.
static void print_results(uint8_t log_id)
{
	unsigned int	slot, slot_min, i;
	uint32_t		rank = 0;

	// Mixer internal stats (enabled with MX_VERBOSE_STATISTICS)
	mixer_print_statistics();

	for (i = 0; i < mx_generation_size; i++)
	{
		if (mixer_stat_slot(i) >= 0) ++rank;
	}

	PRINT_HEADER();
	printf("round=%" PRIu32 " rank=%" PRIu32 " dec=%" PRIu32 " !dec=%" PRIu32 " weak=%" PRIu32
	       " wrong=%" PRIu32 "\n",
	       round, rank, msgs_decoded, msgs_not_decoded, msgs_weak, msgs_wrong);

	msgs_decoded = 0;
	msgs_not_decoded = 0;
	msgs_weak = 0;
	msgs_wrong = 0;

	PRINT_HEADER();
	printf("rank_up_slot=[");
	for (slot_min = 0; 1; )
	{
		slot = -1u;
		for (i = 0; i < mx_generation_size; ++i)
		{
			if (mixer_stat_slot(i) < slot_min)
				continue;

			if (slot > (uint16_t)mixer_stat_slot(i))
				slot = mixer_stat_slot(i);
		}

		if (-1u == slot)
			break;

		for (i = 0; i < mx_generation_size; ++i)
		{
			if (mixer_stat_slot(i) == slot)
				printf("%u;", slot);
		}

		slot_min = slot + 1;
	}
	printf("]\n");

	PRINT_HEADER();
	printf("rank_up_row=[");
	for (slot_min = 0; 1; )
	{
		slot = -1u;
		for (i = 0; i < mx_generation_size; ++i)
		{
			if (mixer_stat_slot(i) < slot_min)
				continue;

			if (slot > (uint16_t)mixer_stat_slot(i))
				slot = mixer_stat_slot(i);
		}

		if (-1u == slot)
			break;

		for (i = 0; i < mx_generation_size; ++i)
		{
			if (mixer_stat_slot(i) == slot)
				printf("%u;", i);
		}

		slot_min = slot + 1;
	}
	printf("]\n");
}

//**************************************************************************************************

static void initialization(void)
{
	// init platform
	gpi_platform_init();
	gpi_int_enable();

	// Start random number generator (RNG)
	NRF_RNG->INTENCLR = BV_BY_NAME(RNG_INTENCLR_VALRDY, Clear);
	NRF_RNG->CONFIG = BV_BY_NAME(RNG_CONFIG_DERCEN, Enabled);
	NRF_RNG->TASKS_START = 1;

	// enable SysTick timer
	SysTick->LOAD  = -1u;
	SysTick->VAL   = 0;
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	printf("\n");
	printf("========================================\n");
	printf("  Mixer Protocol - Auto Node ID Demo\n");
	printf("========================================\n");
	printf("Hardware initialized\n");
	printf("Compiled at " __DATE__ " " __TIME__ "\n");
	printf("========================================\n");
	printf("\n");
}

//**************************************************************************************************
//***** Global Functions ***************************************************************************

int main()
{
	Gpi_Hybrid_Tick	t_ref;
	unsigned int i;

	// Basic hardware initialization
	initialization();
	
	// ***** PHASE 1: NODE DISCOVERY AND ID ASSIGNMENT *****
	run_discovery_phase();
	
	// Now we have our node ID assigned
	node_id = discovery_state.my_logical_id;
	
	// Re-init RF transceiver for Mixer operation
	gpi_radio_init(MX_PHY_MODE);
	gpi_radio_set_tx_power(gpi_radio_dbm_to_power_level(MX_TX_PWR_DBM));
	
	switch (MX_PHY_MODE)
	{
		case BLE_1M:
		case BLE_2M:
		case BLE_125k:
		case BLE_500k:
			gpi_radio_set_channel(39);
			gpi_radio_ble_set_access_address(~0x8E89BED6);
			break;

		case IEEE_802_15_4:
			gpi_radio_set_channel(26);
			break;

		default:
			printf("ERROR: MX_PHY_MODE is invalid!\n");
			assert(0);
	}
	
	// Stop RNG and seed random number generator
	NRF_RNG->TASKS_STOP = 1;
	uint8_t rng_value = BV_BY_VALUE(RNG_VALUE_VALUE, NRF_RNG->VALUE);
	uint32_t rng_seed = rng_value * gpi_mulu_16x16(TOS_NODE_ID, gpi_tick_fast_native());
	printf("Random seed for Mixer: %" PRIu32"\n", rng_seed);
	mixer_rand_seed(rng_seed);

	// Print Mixer configuration
	printf("\n");
	printf("========================================\n");
	printf("   Mixer Configuration\n");
	printf("========================================\n");
	printf("Num Nodes:       %u\n", mx_num_nodes);
	printf("Generation Size: %u\n", mx_generation_size);
	printf("Payload Size:    %u bytes\n", MX_PAYLOAD_SIZE);
	printf("Round Length:    %u slots\n", MX_ROUND_LENGTH);
	printf("Slot Length:     %lu us\n", GPI_TICK_HYBRID_TO_US2(MX_SLOT_LENGTH));
	printf("Initiator ID:    %u\n", mx_initiator_id);
	printf("========================================\n");
	printf("\n");

	// ***** PHASE 2: MIXER OPERATION *****
	
	// t_ref for first round is now
	t_ref = gpi_tick_hybrid();

	// Main Mixer loop
	for (round = 1; 1; round++)
	{
		uint8_t	data[7];

		printf("Preparing round %" PRIu32 " ...\n", round);

		// init mixer with our assigned node_id
		mixer_init(node_id);

		#if MX_WEAK_ZEROS
			mixer_set_weak_release_slot(WEAK_RELEASE_SLOT);
			mixer_set_weak_return_msg((void*)-1);
		#endif

		// Provide test data messages based on payload distribution
		{
			data[1] = node_id;
			data[2] = TOS_NODE_ID;
			data[3] = round;
			data[4] = round >> 8;
			data[5] = round >> 16;
			data[6] = round >> 24;

			for (i = 0; i < mx_generation_size; i++)
			{
				data[0] = i;

				// Check if this message belongs to us
				if (payload_distribution[i] == TOS_NODE_ID)
				{
					mixer_write(i, data, MIN(sizeof(data), MX_PAYLOAD_SIZE));
				}
			}
		}

		// Arm mixer
		// Coordinator (initiator) or participant
		uint8_t arm_flags = 0;
		
		if (mx_initiator_id == TOS_NODE_ID)
			arm_flags |= MX_ARM_INITIATOR;
		
		if (round == 1)
			arm_flags |= MX_ARM_INFINITE_SCAN;
		
		mixer_arm(arm_flags);

		// Delay initiator a bit to let everyone get ready
		if (mx_initiator_id == TOS_NODE_ID)
		{
			t_ref += 3 * MX_SLOT_LENGTH;
		}

		// Start when deadline reached
		printf("Starting round %" PRIu32 " ...\n", round);
		while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), t_ref) < 0);

		// Run Mixer round
		t_ref = mixer_start();

		// Wait until nominal end of round
		while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), t_ref) < 0);

		// Evaluate received data
		for (i = 0; i < mx_generation_size; i++)
		{
			void *p = mixer_read(i);
			if (NULL == p)
			{
				msgs_not_decoded++;
			}
			else if ((void*)-1 == p)
			{
				msgs_weak++;
			}
			else
			{
				memcpy(data, p, sizeof(data));
				if ((data[0] == i) && (data[2] == payload_distribution[i]))
				{
					msgs_decoded++;
				}
				else
				{
					msgs_wrong++;
				}

				// Use message 0 to check/adapt round number (synchronization)
				if ((0 == i) && (MX_PAYLOAD_SIZE >= 7))
				{
					Generic32	r;

					r.u8_ll = data[3];
					r.u8_lh = data[4];
					r.u8_hl = data[5];
					r.u8_hh = data[6];

					if (1 == round)
					{
						round = r.u32;
						printf("Synchronized to round %" PRIu32 "\n", r.u32);
					}
					else if (r.u32 != round)
					{
						printf("Round mismatch: received %" PRIu32 " <> local %" PRIu32 "! Trying resync ...\n", 
						       r.u32, round);
						round = 0;	// increments to 1 with next round loop iteration
					}
				}
			}
		}

		print_results(node_id);

		// Set start time for next round
		t_ref += MAX(10 * MX_SLOT_LENGTH, GPI_TICK_MS_TO_HYBRID2(1000));
	}

	GPI_TRACE_RETURN(0);
}

//**************************************************************************************************
//**************************************************************************************************