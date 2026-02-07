#include "app_logic.h"
#include "main.h"
#include "canard_dynamic_node_id_client.h"
#include <string.h>
#include <stdio.h>

extern CanardInstance canard_ins;
extern CanardDynIDClient dynid_client;
extern uint8_t node_unique_id[16];
extern float tim2_ticks_per_us;
extern TIM_HandleTypeDef htim2;
extern CAN_HandleTypeDef hcan;

ParamEntry params[PARAM_COUNT] = {
    {"can_node_id", 60.0f, 0.0f, 127.0f, 60.0f, true},
    {"ch0_map", 0.0f, -1.0f, 3.0f, 0.0f, true},
    {"ch1_map", 1.0f, -1.0f, 3.0f, 1.0f, true},
    {"ch2_map", 2.0f, -1.0f, 3.0f, 2.0f, true},
    {"ch3_map", 3.0f, -1.0f, 3.0f, 3.0f, true}
};

int8_t pending_node_id = -1;
bool restart_pending = false;
uint32_t restart_request_time_ms = 0;

// transport stats
static uint64_t stats_transfers_tx = 0;
static uint64_t stats_transfers_rx = 0;
static uint64_t stats_transfer_errors = 0;
static uint64_t stats_frames_tx = 0;
static uint64_t stats_frames_rx = 0;
static uint64_t stats_can_errors = 0;

void stats_inc_frame_tx(void) { stats_frames_tx++; }
void stats_inc_frame_rx(void) { stats_frames_rx++; }
void stats_inc_transfer_tx(void) { stats_transfers_tx++; }
void stats_inc_transfer_rx(void) { stats_transfers_rx++; }
void stats_inc_transfer_error(void) { stats_transfer_errors++; }
void stats_inc_can_error(void) { stats_can_errors++; }

#define PARAM_STORE_MAGIC   0x50415241u // 'PARA'
#define PARAM_STORE_VERSION 1u
#define PARAM_STORE_ADDR    0x0801FC00u // adjust for >64K parts

typedef struct {
    uint32_t magic;
    uint32_t version;
    float values[PARAM_COUNT];
    uint32_t crc;
} ParamStoreImage;

static uint32_t param_crc32(const void* data, size_t len)
{
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

bool save_params_to_flash(void)
{
    ParamStoreImage img;
    img.magic = PARAM_STORE_MAGIC;
    img.version = PARAM_STORE_VERSION;
    for (int i = 0; i < PARAM_COUNT; i++) {
        img.values[i] = params[i].value;
    }
    img.crc = param_crc32(&img, sizeof(img) - sizeof(img.crc));

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = PARAM_STORE_ADDR;
    erase.NbPages = 1;
    uint32_t page_error = 0;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        printf("[Param] Flash erase failed err=0x%lX\r\n", (unsigned long)page_error);
        return false;
    }

    uint32_t* src = (uint32_t*)&img;
    size_t words = sizeof(img) / sizeof(uint32_t);
    for (size_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_STORE_ADDR + i * 4u, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("[Param] Flash program failed idx=%lu\r\n", (unsigned long)i);
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

void load_params_from_flash(void)
{
    const ParamStoreImage* img = (const ParamStoreImage*)PARAM_STORE_ADDR;
    if (img->magic != PARAM_STORE_MAGIC || img->version != PARAM_STORE_VERSION) {
        return;
    }
    uint32_t crc = param_crc32(img, sizeof(*img) - sizeof(img->crc));
    if (crc != img->crc) {
        printf("[Param] CRC mismatch, ignore stored values\r\n");
        return;
    }
    for (int i = 0; i < PARAM_COUNT; i++) {
        params[i].value = img->values[i];
    }
}

static int32_t get_mapped_channel(uint8_t logical_index) {
    if (logical_index >= 4) {
        return -1;
    }
    float v = params[1 + logical_index].value;
    return (int32_t)(v + (v >= 0 ? 0.5f : -0.5f));
}

static uint32_t pulse_us_to_ticks(uint16_t pulse_us) {
    return (uint32_t)((float)pulse_us * tim2_ticks_per_us);
}

void processActuatorCommands(uavcan_equipment_actuator_ArrayCommand* cmd) {
    for (uint8_t i = 0; i < cmd->commands.len; i++) {
        uavcan_equipment_actuator_Command* single_cmd = &cmd->commands.data[i];
        uint16_t pulse_us = 1500 + (uint16_t)(single_cmd->command_value * 500.0f);
        if (pulse_us < 1000) pulse_us = 1000;
        if (pulse_us > 2000) pulse_us = 2000;
        int32_t ch = get_mapped_channel(single_cmd->actuator_id);
        uint32_t ccr = pulse_us_to_ticks(pulse_us);
        if (ch < 0 || ch > 3) {
            continue;
        }
        switch (ch) {
            case 0: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr); break;
            case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr); break;
            case 2: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr); break;
            case 3: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr); break;
            default: break;
        }
    }
}

void processEscRawCommand(uavcan_equipment_esc_RawCommand* raw, uint8_t source_node_id) {
    printf("[ESC] src=%u len=%d\r\n", (unsigned)source_node_id, raw->cmd.len);
    for (uint8_t i = 0; i < raw->cmd.len && i < 4; i++) {
        int16_t rc = raw->cmd.data[i];
        float norm = (float)rc / 8192.0f;
        if (norm < -1.0f) norm = -1.0f;
        if (norm > 1.0f) norm = 1.0f;
        uint16_t pulse = 1500 + (uint16_t)(norm * 500.0f);
        if (pulse < 1000) pulse = 1000;
        if (pulse > 2000) pulse = 2000;
        int32_t ch = get_mapped_channel(i);
        uint32_t ccr = pulse_us_to_ticks(pulse);
        printf("  ESC[%d] rc=%d norm=%.3f pulse=%u us ch=%ld\r\n", i, rc, norm, pulse, (long)ch);
        if (ch < 0 || ch > 3) {
            continue;
        }
        switch (ch) {
            case 0: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr); break;
            case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr); break;
            case 2: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr); break;
            case 3: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr); break;
            default: break;
        }
    }
}

void handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer) {
    printf("[GetNodeInfo] Request Received from Node %d\r\n", transfer->source_node_id);
    uavcan_protocol_GetNodeInfoResponse resp;
    memset(&resp, 0, sizeof(resp));
    read_unique_id(resp.hardware_version.unique_id);
    resp.hardware_version.major = 1;
    resp.hardware_version.minor = 0;
    resp.software_version.major = 1;
    resp.software_version.minor = 0;
    resp.software_version.optional_field_flags = 0;
    resp.software_version.vcs_commit = 0;
    resp.status.uptime_sec = HAL_GetTick() / 1000;
    resp.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    resp.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    const char* name = "org.ardupilot.cantopwm";
    resp.name.len = strlen(name);
    resp.name.data = (uint8_t*)name;
    static uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&resp, buffer);
    int16_t res = canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
    if(res <= 0) {
        printf("[GetNodeInfo] Reply Failed: %d\r\n", res);
    } else {
        printf("[GetNodeInfo] Reply Enqueued, len=%lu\r\n", (unsigned long)len);
    }
}

void handleParamGetSet(CanardInstance* ins, CanardRxTransfer* transfer) {
    static uavcan_protocol_param_GetSetRequest req;
    static uint8_t name_buf[92];
    uint8_t* p_name_buf = name_buf;
    memset(&req, 0, sizeof(req));
    memset(name_buf, 0, sizeof(name_buf));
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, transfer->payload_len, &req, &p_name_buf) < 0) {
        printf("[Param] Decode Failed\r\n");
        return;
    }
    if(req.name.len > 0) {
        printf("[Param] Request Name(len=%d): %.*s\r\n", req.name.len, req.name.len, req.name.data);
    } else {
        printf("[Param] Request Index: %d\r\n", req.index);
    }
    printf("[Param] union_tag=%d int=%lld real=%.3f bool=%d payload_len=%lu\r\n",
           req.value.union_tag,
           (long long)req.value.integer_value,
           req.value.real_value,
           req.value.boolean_value,
           (unsigned long)transfer->payload_len);
    ParamEntry* found = NULL;
    if (req.name.len == 0) {
        if (req.index < PARAM_COUNT) {
            found = &params[req.index];
        }
    } else {
        for (int i=0; i<PARAM_COUNT; i++) {
            if (strncmp((const char*)req.name.data, params[i].name, req.name.len) == 0 &&
                strlen(params[i].name) == (size_t)req.name.len) {
                found = &params[i];
                break;
            }
        }
    }
    if (found && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            found->value = (float)req.value.integer_value;
        } else if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
            found->value = req.value.real_value;
        } else if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE) {
            found->value = req.value.boolean_value ? 1.0f : 0.0f;
        }
        if (found->value < found->min) found->value = found->min;
        if (found->value > found->max) found->value = found->max;
        if (found->is_integer) {
            found->value = (float)((int32_t)(found->value + (found->value >= 0 ? 0.5f : -0.5f)));
        }
        printf("[Param] Set %s = %f\r\n", found->name, found->value);
        if (strcmp(found->name, "can_node_id") == 0) {
            uint8_t new_id = (uint8_t)found->value;
            if (new_id == 0 || (new_id >= CANARD_MIN_NODE_ID && new_id <= CANARD_MAX_NODE_ID)) {
                pending_node_id = (int8_t)new_id;
                printf("[Param] Pending Node ID = %d (will apply after handler)\r\n", new_id);
            } else {
                printf("[Param] Node ID out of range: %d\r\n", new_id);
            }
        }
    }
    static uavcan_protocol_param_GetSetResponse resp;
    memset(&resp, 0, sizeof(resp));
    if (found) {
        resp.name.len = strlen(found->name);
        resp.name.data = (uint8_t*)found->name;
        if (found->is_integer) {
            resp.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            resp.value.integer_value = (int64_t)found->value;
            resp.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            resp.default_value.integer_value = (int64_t)found->def;
            resp.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            resp.min_value.integer_value = (int64_t)found->min;
            resp.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            resp.max_value.integer_value = (int64_t)found->max;
        } else {
            resp.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            resp.value.real_value = found->value;
            resp.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            resp.default_value.real_value = found->def;
            resp.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            resp.min_value.real_value = found->min;
            resp.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            resp.max_value.real_value = found->max;
        }
    } else {
        resp.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        printf("[Param] Not found, reply empty\r\n");
    }
    static uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_param_GetSetResponse_encode(&resp, buffer);
    int16_t res = canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
    if(res <= 0) {
        printf("[Param] Reply Failed: %d\r\n", res);
    } else {
        printf("[Param] Reply len=%lu queued\r\n", (unsigned long)len);
    }
}

void handleParamExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer) {
    static uavcan_protocol_param_ExecuteOpcodeRequest req;
    uint8_t* dyn_buf = NULL;
    memset(&req, 0, sizeof(req));
    bool ok = false;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, transfer->payload_len, &req, &dyn_buf) >= 0) {
        if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
            ok = save_params_to_flash();
            printf("[Param] SAVE opcode result=%d\r\n", ok);
        } else if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
            for (int i = 0; i < PARAM_COUNT; i++) {
                params[i].value = params[i].def;
            }
            pending_node_id = (int8_t)params[0].value;
            ok = save_params_to_flash();
            printf("[Param] ERASE opcode result=%d\r\n", ok);
        }
    }
    uavcan_protocol_param_ExecuteOpcodeResponse resp;
    resp.ok = ok;
    static uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&resp, buffer);
    int16_t res = canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
    if (res <= 0) {
        printf("[Param] ExecuteOpcode reply failed: %d\r\n", res);
    }
}

void handleRestartNode(CanardInstance* ins, CanardRxTransfer* transfer) {
    static uavcan_protocol_RestartNodeRequest req;
    uint8_t* dyn_buf = NULL;
    memset(&req, 0, sizeof(req));
    int32_t dec = uavcan_protocol_RestartNodeRequest_decode(transfer, transfer->payload_len, &req, &dyn_buf);
    bool ok = (dec >= 0) && (req.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER);
    uavcan_protocol_RestartNodeResponse resp;
    resp.ok = ok;
    static uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_RestartNodeResponse_encode(&resp, buffer);
    int16_t res = canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE,
                           UAVCAN_PROTOCOL_RESTARTNODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
    if (res <= 0) {
        printf("[RestartNode] Reply Failed: %d\r\n", res);
        return;
    }
    if (ok) {
        restart_pending = true;
        restart_request_time_ms = HAL_GetTick();
        printf("[RestartNode] Accepted, scheduling reset\r\n");
    } else {
        printf("[RestartNode] Reject magic=0x%llX\r\n", (unsigned long long)req.magic_number);
    }
}

void handleGetTransportStats(CanardInstance* ins, CanardRxTransfer* transfer) {
    (void)transfer;
    uavcan_protocol_GetTransportStatsResponse resp;
    memset(&resp, 0, sizeof(resp));
    resp.transfers_tx = stats_transfers_tx;
    resp.transfers_rx = stats_transfers_rx;
    resp.transfer_errors = stats_transfer_errors;
    static uavcan_protocol_CANIfaceStats iface_stats[UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_CAN_IFACE_STATS_MAX_LENGTH];
    resp.can_iface_stats.len = 1;
    resp.can_iface_stats.data = iface_stats;
    iface_stats[0].frames_tx = stats_frames_tx;
    iface_stats[0].frames_rx = stats_frames_rx;
    iface_stats[0].errors   = stats_can_errors;
    static uint8_t buffer[UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_GetTransportStatsResponse_encode(&resp, buffer);
    int16_t res = canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE,
                           UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
    if (res <= 0) {
        printf("[GetTransportStats] Reply Failed: %d\r\n", res);
    }
}

// Stats increment helpers for TX/RX; called from ISR/main contexts
void app_logic_on_tx_ok(void) {
    stats_frames_tx++;
    stats_transfers_tx++;
}

void app_logic_on_tx_fail(void) {
    stats_transfer_errors++;
    stats_can_errors++;
}

void app_logic_on_rx_handled(int16_t cr) {
    if (cr >= 0) {
        stats_transfers_rx++;
    } else {
        stats_transfer_errors++;
    }
}

void app_logic_on_frame_rx(void) {
    stats_frames_rx++;
}
