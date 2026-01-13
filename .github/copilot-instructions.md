# Copilot Instructions for CANtoPWM

## 1. Project Architecture & Context
- **Type:** Embedded C project for STM32F103 (BluePill/Generic).
- **Generator:** STM32CubeMX (HAL Driver based).
- **Core Function:** UAVCAN (DroneCAN) Node acting as a PWM actuator controller.
- **Build System:** Keil MDK-ARM (`.uvprojx`).

## 2. Critical Development Rules (STM32CubeMX)
- **Code Placement:** **ALWAYS** write code inside `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` blocks. Code outside these blocks will be DELETED by STM32CubeMX regeneration.
- **File Structure:**
  - `Core/Src/main.c`: Contains the main application logic, Libcanard integration, and high-level callbacks.
  - `Core/Src/stm32f1xx_it.c`: Interrupt Service Routines.
  - `Core/Src/can.c`, `tim.c`, `gpio.c`: Peripheral initialization (generated).

## 3. Communication Stack (UAVCAN/Libcanard)
- **Library:** Uses legacy Libcanard (v0/DroneCAN) with static memory allocation.
- **Data Flow (RX):**
  1. CAN RX Interrupt triggers.
  2. `HAL_CAN_RxFifo0MsgPendingCallback` (in `main.c`) reads FIFO.
  3. Frame is passed to `canardHandleRxFrame`.
  4. `shouldAcceptTransfer` filters messages.
  5. `onTransferReception` decodes message and calls application logic.
- **Data Flow (TX):**
  1. `canardBroadcast` or `canardRequestOrRespond` pushes to Libcanard internal queue.
  2. `processCanTxQueue` (in `main.c` loop) polls `canardPeekTxQueue`.
  3. Sends via `HAL_CAN_AddTxMessage`.

## 4. Actuation Logic (PWM)
- **Mapping:** UAVCAN `uavcan.equipment.actuator.ArrayCommand`.
- **Scaling:** Input `command_value` (-1.0 to 1.0) maps to PWM Pulse (1000us to 2000us).
  - Center: 1500us (0.0).
  - Range: +/- 500us.
- **Hardware:** `TIM2` with 4 Channels.
  - `actuator_id` 0 -> `TIM_CHANNEL_1`
  - `actuator_id` 1 -> `TIM_CHANNEL_2`
  - ...

## 5. Development Workflows
- **Debugging:** `printf` is retargeted to `HAL_UART_Transmit` (USART1). Use `printf` for debug logs.
- **Dynamic Node ID:** The node acts as a Dynamic Node ID Client (`canard_dynamic_node_id_client.h`). It waits for allocation before processing actuator commands.
- **Testing:** `run_parsing_test()` and `inject_rx_frame_from_server` in `main.c` allow loopback testing without external CAN hardware.

## 6. Common Patterns
- **Time:** Use `HAL_GetTick()` for millisecond timestamps.
- **CAN Filters:** Configured to accept Broadcasts and targeted messages once Node ID is assigned.
- **Handling Arrays:** Libcanard array decoding requires a pointer to a buffer pointer (e.g., `&p_dynamic_array_buffer`).
