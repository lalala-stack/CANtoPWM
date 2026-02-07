/**
 ******************************************************************************
 * @file           : bootloader_can.h
 * @brief          : DroneCAN CAN protocol header for bootloader
 ******************************************************************************
 */

#ifndef BOOTLOADER_CAN_H
#define BOOTLOADER_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize bootloader CAN and libcanard
bool bootloader_can_init(void);

// Process CAN TX queue
void bootloader_can_process_tx(void);

// CAN RX callback (call from HAL_CAN_RxFifo0MsgPendingCallback)
void bootloader_can_rx_callback(CAN_HandleTypeDef* hcan);

// Bootloader CAN main loop (call in while loop)
void bootloader_can_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_CAN_H */
