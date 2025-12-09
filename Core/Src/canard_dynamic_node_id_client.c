/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#include <canard_dynamic_node_id_client.h>
#include <string.h>
#include "uavcan/protocol/dynamic_node_id/Allocation.h"

void canardDynIDClientInit(CanardDynIDClient* self, const uint8_t* unique_id)
{
    memset(self, 0, sizeof(CanardDynIDClient));
    memcpy(self->unique_id, unique_id, sizeof(self->unique_id));
}

int canardDynIDClientStart(CanardDynIDClient* self,
                           CanardInstance* canard_instance,
                           uint8_t preferred_node_id)
{
    uavcan_protocol_dynamic_node_id_Allocation msg;
    memset(&msg, 0, sizeof(msg));

    msg.node_id = preferred_node_id;
    msg.first_part_of_unique_id = true;
    
    // The unique ID is sent in parts, first part is 6 bytes
    msg.unique_id.len = 6;
    msg.unique_id.data = self->unique_id;

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
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, transfer->payload_len, &msg, NULL) >= 0)
    {
        // Check if the response is for us
        if (memcmp(msg.unique_id.data, self->unique_id, msg.unique_id.len) == 0)
        {
            if (msg.node_id > 0 && msg.node_id <= CANARD_MAX_NODE_ID)
            {
                self->node_id = msg.node_id;
            }
        }
    }
}

uint8_t canardDynIDClientGetNodeID(const CanardDynIDClient* self)
{
    return self->node_id;
}
