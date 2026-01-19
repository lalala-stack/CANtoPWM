/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#ifndef CANARD_DYNAMIC_NODE_ID_CLIENT_H
#define CANARD_DYNAMIC_NODE_ID_CLIENT_H

#include <canard.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default timeout for requests, in milliseconds.
 */
#ifndef CANARD_DYN_ID_CLT_DEFAULT_TIMEOUT_MS
# define CANARD_DYN_ID_CLT_DEFAULT_TIMEOUT_MS     1000
#endif

/**
 * Add this value to the timeout of the second transfer.
 */
#ifndef CANARD_DYN_ID_CLT_BIG_DELTA_MS
# define CANARD_DYN_ID_CLT_BIG_DELTA_MS           300
#endif

/**
 * Add this value to the timeout of the third transfer.
 */
#ifndef CANARD_DYN_ID_CLT_SMALL_DELTA_MS
# define CANARD_DYN_ID_CLT_SMALL_DELTA_MS         20
#endif

/**
 * This structure should be initialized once and shared across all library calls.
 */
typedef struct
{
    uint8_t unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    uint8_t node_id;
    uint8_t transfer_id;
    uint8_t uid_prefix_len; // progressive prefix length: 6, 12, 16
} CanardDynIDClient;

/**
 * Initializes the dynamic node ID client.
 *
 * @param self          Client instance.
 * @param unique_id     Unique ID of the node.
 */
void canardDynIDClientInit(CanardDynIDClient* self, const uint8_t* unique_id);

/**
 * Starts the allocation process.
 * This function should be called once.
 *
 * @param self              Client instance.
 * @param canard_instance   Canard instance.
 * @param preferred_node_id Preferred node ID.
 *
 * @return                  Negative value on error, zero on success.
 */
int canardDynIDClientStart(CanardDynIDClient* self,
                           CanardInstance* canard_instance,
                           uint8_t preferred_node_id);

/**
 * Handles allocation response frames.
 *
 * @param self              Client instance.
 * @param transfer          Received transfer.
 */
void canardDynIDClientHandleAllocationResponse(CanardDynIDClient* self, CanardRxTransfer* transfer);

/**
 * Handles received CAN frames.
 *
 * @param self              Client instance.
 * @param canard_instance   Canard instance.
 * @param frame             Received CAN frame.
 *
 * @return                  Negative value on error, zero on success.
 */
int canardDynIDClientHandleFrame(CanardDynIDClient* self,
                                 CanardInstance* canard_instance,
                                 const CanardCANFrame* frame);

/**
 * Returns the allocated node ID.
 *
 * @param self          Client instance.
 *
 * @return              Allocated node ID, or 0 if not allocated.
 */
uint8_t canardDynIDClientGetNodeID(const CanardDynIDClient* self);

#ifdef __cplusplus
}
#endif
#endif
