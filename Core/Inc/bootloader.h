/**
 ******************************************************************************
 * @file           : bootloader.h
 * @brief          : DroneCAN OTA Bootloader header
 ******************************************************************************
 */

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Check if bootloader mode is requested (via backup register)
bool bootloader_is_requested(void);

// Clear the bootloader request flag
void bootloader_clear_flag(void);

// Check if valid firmware exists in application flash
bool bootloader_check_firmware(void);

// Erase application flash area
bool bootloader_erase_flash(void);

// Write firmware data to flash
bool bootloader_write_flash(uint32_t addr, const uint8_t* data, uint16_t len);

// Jump to application
void bootloader_jump_to_app(void);

// Initialize CAN for bootloader mode
bool bootloader_can_init(void);

// Bootloader main entry point
void bootloader_main(void);

// Bootloader entry point (call from main.c before app initialization)
void bootloader_entry(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_H */
