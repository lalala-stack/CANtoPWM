/**
 ******************************************************************************
 * @file           : bootloader_can.c
 * @brief          : DroneCAN CAN protocol implementation for bootloader
 * @description    : Handles DroneCAN file read operations for OTA update
 ******************************************************************************
 */

// Standard includes
#include <string.h>
#include <stdio.h>

// libcanard includes (from Core/Inc) - must be first
#include "canard.h"
#include "canard_internals.h"

// DroneCAN protocol includes (from Core/include)
#include "uavcan/protocol/GetNodeInfo.h"
#include "uavcan/protocol/NodeStatus.h"
#include "uavcan/protocol/file/Read.h"
#include "uavcan/protocol/file/Error.h"
#include "uavcan/protocol/file/Path.h"
#include "uavcan/protocol/dynamic_node_id/Allocation.h"

// Local includes
#include "bootloader_can.h"
#include "bootloader.h"

// Bootloader node ID (will be dynamically allocated or use fixed ID)
#define BOOTLOADER_NODE_ID_DEFAULT  100u
#define BOOTLOADER_NODE_UNIQUE_ID_OFFSET 0xA5

// CANARD buffer configuration
#define CANARD_MEMORY_POOL_SIZE    4096u

static CanardInstance canard_ins;
static uint8_t canard_memory_pool[CANARD_MEMORY_POOL_SIZE];
static uint8_t bootloader_node_id = BOOTLOADER_NODE_ID_DEFAULT;
static uint8_t unique_id[16];

// External CAN handle declaration (from main.c)
extern CAN_HandleTypeDef hcan;

// File read state
typedef struct {
    uint32_t file_offset;
    uint32_t file_size;
    uint32_t bytes_written;
    uint32_t source_node_id;
    bool active;
} FileReadState;

static FileReadState file_read_state;

// Get node unique ID
static void bootloader_read_unique_id(uint8_t* out_uid) {
    const uint32_t* uid_reg = (const uint32_t*)0x1FFFF7E8;
    uint8_t uid_buf[12];
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t word = uid_reg[i];
        uid_buf[i*4 + 0] = (word >> 0) & 0xFF;
        uid_buf[i*4 + 1] = (word >> 8) & 0xFF;
        uid_buf[i*4 + 2] = (word >> 16) & 0xFF;
        uid_buf[i*4 + 3] = (word >> 24) & 0xFF;
    }
    // Pad with bootloader offset to differentiate from app
    memset(out_uid, BOOTLOADER_NODE_UNIQUE_ID_OFFSET, 16);
    memcpy(out_uid, uid_buf, 12);
}

// Check if we should accept a transfer
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                uint64_t* out_data_type_signature,
                                uint16_t data_type_id,
                                CanardTransferType transfer_type,
                                uint8_t source_node_id) {
    (void)ins;
    (void)source_node_id;

    // GetNodeInfo Request (Service ID=1)
    if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID &&
        transfer_type == CanardTransferTypeRequest) {
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    }

    // File Read Request (Service ID=41)
    if (data_type_id == UAVCAN_PROTOCOL_FILE_READ_ID &&
        transfer_type == CanardTransferTypeRequest) {
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
        return true;
    }

    // Dynamic node ID allocation
    if (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID &&
        transfer_type == CanardTransferTypeBroadcast) {
        *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
        return true;
    }

    return false;
}

// Handle GetNodeInfo request
static void handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer) {
    uavcan_protocol_GetNodeInfoResponse resp;
    memset(&resp, 0, sizeof(resp));

    bootloader_read_unique_id(resp.hardware_version.unique_id);
    resp.hardware_version.major = 1;
    resp.hardware_version.minor = 0;
    resp.software_version.major = 1;
    resp.software_version.minor = 0;
    resp.software_version.optional_field_flags = 0;
    resp.software_version.vcs_commit = 0;
    resp.status.uptime_sec = HAL_GetTick() / 1000;
    resp.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    resp.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    const char* name = "org.ardupilot.cantopwm.bootloader";
    resp.name.len = strlen(name);
    resp.name.data = (uint8_t*)name;

    static uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&resp, buffer);

    canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
}

// Handle File Read request
static void handleFileRead(CanardInstance* ins, CanardRxTransfer* transfer) {
    static uavcan_protocol_file_ReadRequest req;
    static uint8_t path_buf[UAVCAN_PROTOCOL_FILE_PATH_PATH_MAX_LENGTH];
    uint8_t* dyn_buf = path_buf;

    memset(&req, 0, sizeof(req));
    int32_t dec = uavcan_protocol_file_ReadRequest_decode(
        transfer, transfer->payload_len, &req, &dyn_buf);

    uavcan_protocol_file_ReadResponse resp;
    memset(&resp, 0, sizeof(resp));

    if (dec >= 0) {
        // Start or continue file read
        if (!file_read_state.active) {
            file_read_state.active = true;
            file_read_state.file_offset = 0;
            file_read_state.bytes_written = 0;
            file_read_state.source_node_id = transfer->source_node_id;
            printf("[Bootloader] File read started: %.*s\r\n",
                   req.path.path.len, req.path.path.data);
        }

        // Fixed chunk size for bootloader mode
        uint16_t chunk_size = 256u;

        // In a real implementation, we would read the actual file data here
        // For now, we simulate by reading from a buffer or request more data
        resp.error.value = 0;  // OK
        resp.data.data = path_buf;  // Reuse buffer as temp storage
        resp.data.len = 0;  // TODO: Fill with actual data

        printf("[Bootloader] File read request: offset=%lu, chunk_size=%u\r\n",
               (unsigned long)req.offset, chunk_size);
    } else {
        resp.error.value = 2;  // ERROR_UNKNOWN
    }

    static uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_file_ReadResponse_encode(&resp, buffer);

    canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
}

// Handle received transfer
static void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID &&
        transfer->transfer_type == CanardTransferTypeRequest) {
        handleGetNodeInfo(ins, transfer);
    }
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_FILE_READ_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleFileRead(ins, transfer);
    }

    canardReleaseRxTransferPayload(ins, transfer);
}

// Publish NodeStatus
static void publishNodeStatus(void) {
    uavcan_protocol_NodeStatus msg;
    msg.uptime_sec = HAL_GetTick() / 1000;
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    msg.sub_mode = 0;
    msg.vendor_specific_status_code = 0;

    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint32_t len = uavcan_protocol_NodeStatus_encode(&msg, buffer);

    static uint8_t transfer_id = 0;
    canardBroadcast(&canard_ins,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

// Process CAN TX queue
void bootloader_can_process_tx(void) {
    const CanardCANFrame* tx_frame = canardPeekTxQueue(&canard_ins);

    while (tx_frame != NULL) {
        CAN_TxHeaderTypeDef tx_header = {
            .ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK,
            .IDE = CAN_ID_EXT,
            .RTR = CAN_RTR_DATA,
            .DLC = tx_frame->data_len
        };

        uint32_t mailbox;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
            break;
        }

        if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_frame->data, &mailbox) == HAL_OK) {
            canardPopTxQueue(&canard_ins);
        } else {
            break;
        }

        tx_frame = canardPeekTxQueue(&canard_ins);
    }
}

// Initialize bootloader CAN
bool bootloader_can_init(void) {
    // Get unique ID
    bootloader_read_unique_id((uint8_t*)unique_id);

    // Initialize CANARD
    canardInit(&canard_ins,
               canard_memory_pool,
               sizeof(canard_memory_pool),
               onTransferReception,
               shouldAcceptTransfer,
               NULL);

    // Use fixed node ID for bootloader (simplifies firmware update flow)
    bootloader_node_id = BOOTLOADER_NODE_ID_DEFAULT;
    canardSetLocalNodeID(&canard_ins, bootloader_node_id);

    printf("[Bootloader CAN] Node ID = %u\r\n", bootloader_node_id);

    // Reinitialize CAN
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 2;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        return false;
    }

    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return false;
    }

    return true;
}

// CAN RX callback (called from main.c)
void bootloader_can_rx_callback(CAN_HandleTypeDef* hcan_ptr) {
    CanardCANFrame rx_frame;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        rx_frame.id = (rx_header.ExtId & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
        if (rx_header.RTR == CAN_RTR_REMOTE) {
            rx_frame.id |= CANARD_CAN_FRAME_RTR;
        }
        rx_frame.data_len = rx_header.DLC;
        memcpy(rx_frame.data, rx_data, rx_header.DLC);

        uint64_t timestamp_us = HAL_GetTick() * 1000;
        canardHandleRxFrame(&canard_ins, &rx_frame, timestamp_us);
    }
}

// Bootloader CAN loop (call this from main while loop)
void bootloader_can_loop(void) {
    // Process TX queue
    bootloader_can_process_tx();

    // Publish NodeStatus periodically (1Hz)
    static uint32_t last_status_time = 0;
    if (HAL_GetTick() - last_status_time >= 1000) {
        publishNodeStatus();
        last_status_time = HAL_GetTick();
    }

    // Cleanup stale transfers
    uint64_t timestamp = HAL_GetTick() * 1000;
    canardCleanupStaleTransfers(&canard_ins, timestamp);

    HAL_Delay(1);
}
