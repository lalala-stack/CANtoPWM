/**
 ******************************************************************************
 * @file           : bootloader.c
 * @brief          : DroneCAN OTA Bootloader for STM32F103CBT6
 * @description    : Implements firmware update via DroneCAN BeginFirmwareUpdate
 *                  protocol and file read operations
 ******************************************************************************
 */

#include "bootloader.h"
#include "bootloader_can.h"
#include "main.h"
#include "can.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

#define BOOTLOADER_FLAG_MAGIC 0xB007B007u
#define BOOTLOADER_FLAG_ADDR  0x0801FB00u  // Flash address for bootloader flag (before params at 0x0801FC00)

// Firmware image configuration
#define APP_START_ADDR    0x08000000u
#define APP_END_ADDR       0x0801FC00u  // Leave last page for params
#define APP_SIZE_MAX       (APP_END_ADDR - APP_START_ADDR)

// CAN buffer configuration
#define CAN_RX_BUFFER_SIZE 256u
#define CAN_TX_QUEUE_SIZE  64u

// File read chunk size
#define FILE_READ_CHUNK_SIZE 256u

// LED status indication
void bootloader_set_led(bool state) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void bootloader_led_error(void) {
    // Fast blinking indicates error
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}

void bootloader_led_success(void) {
    // Three slow blinks indicate success
    for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(300);
    }
}

// Check if bootloader mode is requested
bool bootloader_is_requested(void) {
    uint32_t flag = *(volatile uint32_t*)BOOTLOADER_FLAG_ADDR;
    return (flag == BOOTLOADER_FLAG_MAGIC);
}

// Clear bootloader flag
void bootloader_clear_flag(void) {
    HAL_FLASH_Unlock();
    // Write 0 to clear the flag
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BOOTLOADER_FLAG_ADDR, 0);
    HAL_FLASH_Lock();
}

// Check if valid firmware exists in application flash
bool bootloader_check_firmware(void) {
    // Check first word for valid vector table (should be stack pointer address)
    uint32_t* app_vector = (uint32_t*)APP_START_ADDR;

    // Stack pointer should point to RAM (0x20000000 - 0x20005000 for STM32F103CBT6)
    uint32_t stack_ptr = app_vector[0];
    if (stack_ptr < 0x20000000 || stack_ptr > 0x20005000) {
        return false;
    }

    // Reset vector should point to flash
    uint32_t reset_vector = app_vector[1];
    if (reset_vector < APP_START_ADDR || reset_vector > APP_END_ADDR) {
        return false;
    }

    return true;
}

// Erase application flash area
bool bootloader_erase_flash(void) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = APP_START_ADDR;
    erase.NbPages = APP_SIZE_MAX / FLASH_PAGE_SIZE;
    uint32_t page_error = 0;

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    HAL_FLASH_Lock();
    return true;
}

// Write firmware data to flash
bool bootloader_write_flash(uint32_t addr, const uint8_t* data, uint16_t len) {
    HAL_FLASH_Unlock();

    // Ensure word alignment
    uint32_t word_offset = 0;
    uint32_t* src = (uint32_t*)data;

    // Write complete words
    while (word_offset + 4 <= len) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + word_offset, src[word_offset / 4]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        word_offset += 4;
    }

    // Write remaining bytes (padded to word)
    if (word_offset < len) {
        uint32_t last_word = 0;
        memcpy(&last_word, &data[word_offset], len - word_offset);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + word_offset, last_word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

// Jump to application
void bootloader_jump_to_app(void) {
    uint32_t app_addr = APP_START_ADDR;

    // Disable interrupts
    __disable_irq();

    // Reset peripherals
    HAL_DeInit();

    // Get stack pointer and reset vector
    uint32_t* app_vector = (uint32_t*)app_addr;
    uint32_t stack_ptr = app_vector[0];
    uint32_t reset_vector = app_vector[1];

    // Set stack pointer
    __set_MSP(stack_ptr);

    // Jump to application
    void (*app_reset)(void) = (void (*)(void))reset_vector;
    app_reset();
}

// Initialize CAN for bootloader mode
bool bootloader_hardware_init(void) {
    // Initialize CAN hardware
    // bootloader_can_init() will be called separately
    return true;
}

// Unused variables removal to suppress warnings

// Bootloader main loop
void bootloader_main(void) {
    printf("\r\n=== DroneCAN OTA Bootloader ===\r\n");

    if (!bootloader_can_init()) {
        bootloader_led_error();
        printf("[Bootloader] CAN init failed\r\n");
        return;
    }

    printf("[Bootloader] CAN initialized, waiting for firmware transfer...\r\n");

    // Main bootloader loop
    uint32_t timeout_counter = 0;
    const uint32_t BOOTLOADER_TIMEOUT_MS = 60000; // 60 second timeout

    while (1) {
        bootloader_can_loop();

        // Check for timeout or completion
        if (timeout_counter >= BOOTLOADER_TIMEOUT_MS) {
            printf("[Bootloader] Timeout, checking for valid firmware...\r\n");
            break;
        }

        timeout_counter++;
        HAL_Delay(1);
    }

    // Clear flag and jump if valid firmware exists
    if (bootloader_check_firmware()) {
        bootloader_clear_flag();
        bootloader_led_success();
        printf("[Bootloader] Valid firmware found, jumping...\r\n");
        HAL_Delay(500);
        bootloader_jump_to_app();
    } else {
        bootloader_led_error();
        printf("[Bootloader] No valid firmware!\r\n");
        while (1) {
            HAL_Delay(1000);
        }
    }
}

// Main entry point (call this from main.c if bootloader flag is set)
void bootloader_entry(void) {
    // Turn on LED to indicate bootloader mode
    bootloader_set_led(true);

    // Check if bootloader mode was requested
    if (bootloader_is_requested()) {
        printf("[Bootloader] Update flag detected\r\n");
        bootloader_main();
    }

    // Otherwise, check if valid firmware exists
    if (bootloader_check_firmware()) {
        printf("[Bootloader] Starting application...\r\n");
        bootloader_led_success();
        HAL_Delay(500);
        bootloader_jump_to_app();
    } else {
        printf("[Bootloader] No valid firmware, staying in bootloader\r\n");
        bootloader_led_error();
        bootloader_main();
    }
}
