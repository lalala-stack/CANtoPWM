#ifndef APP_LOGIC_H
#define APP_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "canard.h"
#include "uavcan/equipment/actuator/ArrayCommand.h"
#include "uavcan/equipment/esc/RawCommand.h"
#include "uavcan/protocol/GetNodeInfo.h"
#include "uavcan/protocol/param/GetSet.h"
#include "uavcan/protocol/param/ExecuteOpcode.h"
#include "uavcan/protocol/RestartNode.h"
#include "uavcan/protocol/GetTransportStats.h"

typedef struct {
    const char* name;
    float value;
    float min;
    float max;
    float def;
    bool is_integer;
} ParamEntry;

#define PARAM_COUNT 5

extern ParamEntry params[PARAM_COUNT];
extern int8_t pending_node_id;
extern bool restart_pending;
extern uint32_t restart_request_time_ms;

void load_params_from_flash(void);
bool save_params_to_flash(void);

void handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer);
void handleParamGetSet(CanardInstance* ins, CanardRxTransfer* transfer);
void handleParamExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer);
void handleRestartNode(CanardInstance* ins, CanardRxTransfer* transfer);
void handleGetTransportStats(CanardInstance* ins, CanardRxTransfer* transfer);
void processActuatorCommands(uavcan_equipment_actuator_ArrayCommand* cmd);
void processEscRawCommand(uavcan_equipment_esc_RawCommand* raw, uint8_t source_node_id);

// Transport stats helpers
void stats_inc_frame_tx(void);
void stats_inc_frame_rx(void);
void stats_inc_transfer_tx(void);
void stats_inc_transfer_rx(void);
void stats_inc_transfer_error(void);
void stats_inc_can_error(void);

// Platform helper from main.c
void read_unique_id(uint8_t* out_uid);

#endif // APP_LOGIC_H
