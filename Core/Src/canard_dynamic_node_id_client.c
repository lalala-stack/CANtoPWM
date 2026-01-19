/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#include <canard_dynamic_node_id_client.h>
#include <string.h>
#include <stdio.h>
#include "uavcan/protocol/dynamic_node_id/Allocation.h"

void canardDynIDClientInit(CanardDynIDClient* self, const uint8_t* unique_id)
{
    memset(self, 0, sizeof(CanardDynIDClient));
    memcpy(self->unique_id, unique_id, sizeof(self->unique_id));
    // Tracks how many UID bytes have been echoed by allocator so far; start at 0
    self->uid_prefix_len = 0U;
}

int canardDynIDClientStart(CanardDynIDClient* self,
                           CanardInstance* canard_instance,
                           uint8_t preferred_node_id)
{
    uavcan_protocol_dynamic_node_id_Allocation msg;
    memset(&msg, 0, sizeof(msg));

    // Per spec on Classic CAN, each request must be single-frame: <=6 bytes.
    // Send next chunk starting at current offset (uid_prefix_len bytes already echoed by allocator).
    uint8_t offset = self->uid_prefix_len;
    if (offset > sizeof(self->unique_id)) {
        offset = sizeof(self->unique_id);
    }

    // If allocator already echoed full UID, do not send more follow-ups; wait for allocation
    if (offset >= sizeof(self->unique_id)) {
        return 0;
    }
    uint8_t chunk_len = (uint8_t)((sizeof(self->unique_id) - offset) > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST ?
                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST :
                                  (sizeof(self->unique_id) - offset));

    msg.node_id = preferred_node_id;
    msg.first_part_of_unique_id = (offset == 0U);
    msg.unique_id.len = chunk_len;
    msg.unique_id.data = &self->unique_id[offset];

    printf("[DynID] Req tid=%u prefix_len=%u prefer=%u\r\n",
           (unsigned)self->transfer_id,
           (unsigned)msg.unique_id.len,
           (unsigned)preferred_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
    uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, buffer);

    return canardBroadcast(canard_instance,
                           UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                           UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                           &self->transfer_id,
                           CANARD_TRANSFER_PRIORITY_LOW,
                           buffer,
                           len
#if CANARD_ENABLE_DEADLINE
                           ,0
#endif
#if CANARD_MULTI_IFACE
                           ,0
#endif
#if CANARD_ENABLE_CANFD
                           ,false
#endif
                           );
}

void canardDynIDClientHandleAllocationResponse(CanardDynIDClient* self, CanardRxTransfer* transfer)
{
    if (self->node_id != 0)
    {
        return;
    }

    uavcan_protocol_dynamic_node_id_Allocation msg;
    
    // --- FIX BEGIN: 提供缓冲区给动态数组 ---
    uint8_t unique_id_buffer[16]; 
    uint8_t* dyn_arr_buf = unique_id_buffer;

    // 传入 &dyn_arr_buf
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, transfer->payload_len, &msg, &dyn_arr_buf) >= 0)
    // --- FIX END ---
    {
        printf("[DynID] Decoded alloc: src=%u node_id=%u len=%u first=%u uid0=%02X uid1=%02X uid2=%02X uid3=%02X\r\n",
               (unsigned)transfer->source_node_id,
               (unsigned)msg.node_id,
               (unsigned)msg.unique_id.len,
               (unsigned)msg.first_part_of_unique_id,
               (unsigned)msg.unique_id.len ? msg.unique_id.data[0] : 0,
               (unsigned)msg.unique_id.len > 1 ? msg.unique_id.data[1] : 0,
               (unsigned)msg.unique_id.len > 2 ? msg.unique_id.data[2] : 0,
               (unsigned)msg.unique_id.len > 3 ? msg.unique_id.data[3] : 0);
        // Check if the response is for us
        if (memcmp(msg.unique_id.data, self->unique_id, msg.unique_id.len) == 0)
        {
            if (msg.node_id > 0 && msg.node_id <= CANARD_MAX_NODE_ID)
            {
                self->node_id = msg.node_id;
                printf("[DynID] Got allocation: node_id=%u len=%u first=%u\r\n",
                       (unsigned)msg.node_id,
                       (unsigned)msg.unique_id.len,
                       (unsigned)msg.first_part_of_unique_id);
                self->uid_prefix_len = 0U;
            }
            else {
                // advance offset to echoed length (allocator may echo cumulative length), capped at UID size
                uint8_t next_off = msg.unique_id.len;
                if (next_off > sizeof(self->unique_id)) {
                    next_off = sizeof(self->unique_id);
                }
                self->uid_prefix_len = next_off;
            }
        } else {
            printf("[DynID] UID mismatch (resp len=%u)\r\n", (unsigned)msg.unique_id.len);
        }
    }
}

uint8_t canardDynIDClientGetNodeID(const CanardDynIDClient* self)
{
    return self->node_id;
}
