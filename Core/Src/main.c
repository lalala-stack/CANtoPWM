/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#pragma anon_unions
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "canard.h"
#include "canard_internals.h"
#include "uavcan/equipment/actuator/ArrayCommand.h"
#include "uavcan/equipment/esc/RawCommand.h"
#include "uavcan/protocol/NodeStatus.h"
#include "uavcan/protocol/GetNodeInfo.h"
#include "uavcan/protocol/param/GetSet.h"
#include "uavcan/protocol/param/ExecuteOpcode.h"
#include "uavcan/protocol/RestartNode.h"
#include "uavcan/protocol/GetTransportStats.h"
#include "uavcan/protocol/file/BeginFirmwareUpdate.h"
#include <stdio.h>
#include "canard_dynamic_node_id_client.h"
#include "app_logic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 函数声明
bool shouldAcceptTransfer(const CanardInstance* ins,
                         uint64_t* out_data_type_signature,
                         uint16_t data_type_id,
                         CanardTransferType transfer_type,
                         uint8_t source_node_id);
void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer);
void processCanTxQueue(void);
int16_t sendActuatorCommand(CanardInstance* ins, uint8_t actuator_id, float value);
int16_t sendNodeStatusRequest(CanardInstance* ins, uint8_t target_node_id);
void publishNodeStatus(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BOOTLOADER_FLAG_MAGIC 0xB007B007u
#define BOOTLOADER_FLAG_REG   (BKP->DR1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// CANARD
CanardInstance canard_ins;
uint8_t canard_memory_pool[4096]; //内存池

// Dynamic Node ID client
CanardDynIDClient dynid_client;
uint8_t node_unique_id[16];

//传输ID计数器
static uint8_t actuator_cmd_transfer_id = 0;

//接受到的数据储存
static uavcan_equipment_actuator_ArrayCommand rx_actuator_cmd;

// 参数存储 (演示用)
float tim2_ticks_per_us = 1.0f; // updated at init to scale 1-2ms to CCR
static bool dyn_allocation_active = false;
static uint32_t dyn_last_req_ms = 0;
static bool dyn_followup_pending = false; // schedule immediate follow-up chunk after allocator echo
static uint32_t dyn_followup_deadline_ms = 0;
static bool bootloader_update_requested = false;
static volatile uint32_t can_rx_irq_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void read_unique_id(uint8_t* out_uid);
static void update_tim2_tick_scale(void);
static void start_dynamic_allocation(void);
static void set_bootloader_flag(void);
static void handleBeginFirmwareUpdate(CanardInstance* ins, CanardRxTransfer* transfer);
static void boot_led_flash(void);
static bool wait_mailbox_result(uint32_t mailbox, uint32_t timeout_ms, bool* out_txok, uint32_t* out_tsr_snapshot);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void read_unique_id(uint8_t* out_uid)
{
    // STM32F1xx series has a 96-bit unique ID starting at this address
    const uint32_t* uid_reg = (const uint32_t*)0x1FFFF7E8;
    uint8_t uid_buf[12];
    for (uint8_t i = 0; i < 3; i++)
    {
        uint32_t word = uid_reg[i];
        uid_buf[i*4 + 0] = (word >> 0) & 0xFF;
        uid_buf[i*4 + 1] = (word >> 8) & 0xFF;
        uid_buf[i*4 + 2] = (word >> 16) & 0xFF;
        uid_buf[i*4 + 3] = (word >> 24) & 0xFF;
    }
    // The unique ID for allocation is 16 bytes, we pad the 12-byte MCU ID with zeros
    memset(out_uid, 0, 16);
    memcpy(out_uid, uid_buf, 12);
}

static void set_bootloader_flag(void)
{
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    BOOTLOADER_FLAG_REG = BOOTLOADER_FLAG_MAGIC;
    HAL_PWR_DisableBkUpAccess();
}

static void handleBeginFirmwareUpdate(CanardInstance* ins, CanardRxTransfer* transfer)
{
    static uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    static uint8_t path_buf[UAVCAN_PROTOCOL_FILE_PATH_PATH_MAX_LENGTH];
    uint8_t* dyn_buf = path_buf;
    memset(&req, 0, sizeof(req));
    int32_t dec = uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(
        transfer, transfer->payload_len, &req, &dyn_buf);

    uavcan_protocol_file_BeginFirmwareUpdateResponse resp;
    memset(&resp, 0, sizeof(resp));
    resp.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    static uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&resp, buffer);
    (void)canardRequestOrRespond(ins, transfer->source_node_id,
                                 UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                                 UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                                 &transfer->transfer_id,
                                 transfer->priority,
                                 CanardResponse,
                                 buffer,
                                 len);

    if (dec >= 0) {
        bootloader_update_requested = true;
        restart_pending = true;
        restart_request_time_ms = HAL_GetTick();
        printf("[FWU] BeginFirmwareUpdate accepted\r\n");
    } else {
        printf("[FWU] Decode fail: %ld\r\n", (long)dec);
    }
}

// Fast LED blink on boot to indicate firmware ready state
static void boot_led_flash(void)
{
    for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(50);
    }
}

// --- 辅助工具：虚拟服务器实例 ---
// 用于生成正确的 CAN 帧（自动处理多帧、CRC、尾字节）
void inject_rx_frame_from_server(CanardInstance* server_ins, uint64_t timestamp_usec) {
    for (const CanardCANFrame* tx_frame = canardPeekTxQueue(server_ins); 
         tx_frame != NULL; 
         tx_frame = canardPeekTxQueue(server_ins)) 
    {
        // 将服务器发出的帧，注入到主实例的接收端
        canardHandleRxFrame(&canard_ins, tx_frame, timestamp_usec);
        // 从服务器队列移除
        canardPopTxQueue(server_ins);
    }
}

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint16_t led_cnt = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 读取MCU唯一ID
  read_unique_id(node_unique_id);
  printf("UID: ");
  for(int i=0; i<16; i++) printf("%02X ", node_unique_id[i]);
  printf("\r\n");

  // 加载已保存的参数（若校验通过），否则保持默认值
  load_params_from_flash();

  // 启动TIM2的4个PWM通道
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // 根据当前 PCLK1/预分频计算 TIM2 每微秒的计数，确保无论周期如何调整都能输出 1-2ms 脉宽
  update_tim2_tick_scale();

    // 开机快速闪烁，提示固件已进入主程序
    boot_led_flash();

  // 初始化libcanard
  canardInit(&canard_ins,
             canard_memory_pool,
             sizeof(canard_memory_pool),
             onTransferReception,
             shouldAcceptTransfer,
             NULL);

  canardDynIDClientInit(&dynid_client, node_unique_id);
  uint8_t initial_node_id = (uint8_t)(params[0].value + 0.5f);
  if (initial_node_id == 0) {
      start_dynamic_allocation();
  } else {
      canardSetLocalNodeID(&canard_ins, initial_node_id);
  }

  // 初始化CAN过滤器（放宽，接收所有包）
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
  HAL_CAN_ConfigFilter(&hcan, &filter);

  // 启动CAN
  HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan,
                                                             CAN_IT_RX_FIFO0_MSG_PENDING |
                                                             CAN_IT_ERROR_WARNING |
                                                             CAN_IT_ERROR_PASSIVE |
                                                             CAN_IT_BUSOFF |
                                                             CAN_IT_LAST_ERROR_CODE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 固定ID模式：直接处理发送队列
    processCanTxQueue();

    // 如果有待应用的节点ID，移到主循环中执行，避免在服务传输过程中切换导致分片被拒绝
    if (pending_node_id >= 0 && pending_node_id <= CANARD_MAX_NODE_ID) {
        uint8_t new_id = (uint8_t)pending_node_id;
        if (new_id == 0) {
            start_dynamic_allocation();
        } else {
            dyn_allocation_active = false;
            canardSetLocalNodeID(&canard_ins, new_id);
        }
        params[0].value = (float)new_id;
        pending_node_id = -1;
        printf("[Param] Applied Node ID = %d\r\n", new_id);
    }

    if (dyn_allocation_active) {
        uint8_t allocated = canardDynIDClientGetNodeID(&dynid_client);
        if (allocated != 0 && allocated <= CANARD_MAX_NODE_ID) {
            canardSetLocalNodeID(&canard_ins, allocated);
            dyn_allocation_active = false;
            params[0].value = (float)allocated;
            printf("[DynID] Allocated Node ID = %d\r\n", allocated);
            dyn_followup_pending = false;
        } else if (dyn_followup_pending && HAL_GetTick() >= dyn_followup_deadline_ms && dynid_client.uid_prefix_len < sizeof(node_unique_id)) {
            (void)canardDynIDClientStart(&dynid_client, &canard_ins, 0);
            dyn_last_req_ms = HAL_GetTick();
            dyn_followup_pending = false;
        } else if (HAL_GetTick() - dyn_last_req_ms >= 1000 && dynid_client.uid_prefix_len < sizeof(node_unique_id)) {
            (void)canardDynIDClientStart(&dynid_client, &canard_ins, 0);
            dyn_last_req_ms = HAL_GetTick();
        }
    }
    
    // 发送心跳包 (1Hz)
    static uint32_t last_node_status_time = 0;
    if (HAL_GetTick() - last_node_status_time >= 1000)
    {
        publishNodeStatus();
        last_node_status_time = HAL_GetTick();
    }

    // 清理过期的传输
    uint64_t timestamp = HAL_GetTick() * 1000; // 转换为微秒
    canardCleanupStaleTransfers(&canard_ins, timestamp);

    // 每秒打印一次 RX 中断计数，快速判断是否有任何合法 CAN 帧进入 FIFO
    static uint32_t rx_mon_last_ms = 0;
    static uint32_t rx_mon_last_cnt = 0;
    if (HAL_GetTick() - rx_mon_last_ms >= 1000U) {
        uint32_t now_cnt = can_rx_irq_count;
        uint32_t delta = now_cnt - rx_mon_last_cnt;
        printf("[CAN RX] irq/s=%lu total=%lu\r\n",
               (unsigned long)delta,
               (unsigned long)now_cnt);
        rx_mon_last_cnt = now_cnt;
        rx_mon_last_ms = HAL_GetTick();
    }

    // 如收到 RestartNode 请求，等回复排队后再复位，避免截断响应
    if (restart_pending) {
        bool tx_empty = (canardPeekTxQueue(&canard_ins) == NULL);
        bool waited = (HAL_GetTick() - restart_request_time_ms) > 200;
        if (tx_empty || waited) {
            if (bootloader_update_requested) {
                set_bootloader_flag();
            }
            NVIC_SystemReset();
        }
    }
    
    HAL_Delay(1); // 短暂延迟

    // 简单心跳 LED 闪烁
    if (led_cnt >= 1000) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        led_cnt = 0;
    } else {
        led_cnt++;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
        /* USER CODE END 3 */
}

/**
    * @brief System Clock Configuration
    * @retval None
    */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

// 决定是否接受某个传输的回调函数
bool shouldAcceptTransfer(const CanardInstance* ins,
                         uint64_t* out_data_type_signature,
                         uint16_t data_type_id,
                         CanardTransferType transfer_type,
                         uint8_t source_node_id) {
    
    // Debug打印关闭以减少中断内耗时，避免丢帧

    (void)ins;
		(void)source_node_id;

		// 在获得节点ID之前，只接受动态节点ID分配消息
    if (canardGetLocalNodeID(&canard_ins) == CANARD_BROADCAST_NODE_ID)
    {
        if (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID &&
            transfer_type == CanardTransferTypeBroadcast)
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }

        // 允许在未分配 ID 时也响应 GetNodeInfo，便于上位机显示节点名
        if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID &&
            transfer_type == CanardTransferTypeRequest)
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
            return true;
        }
        return false;
    }

    // 根据数据类型ID决定是否接受
    if (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
        return true;
    }

    // ESC RawCommand (Message ID=1030)
    if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
        return true;
    }

    // GetTransportStats Request (Service ID=4)
    if (data_type_id == UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID && transfer_type == CanardTransferTypeRequest) {
        *out_data_type_signature = UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE;
        return true;
    }

    // RestartNode Request (Service ID=5)
    if (data_type_id == UAVCAN_PROTOCOL_RESTARTNODE_ID && transfer_type == CanardTransferTypeRequest) {
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
    }

    // BeginFirmwareUpdate Request (Service ID=40)
    if (data_type_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID && transfer_type == CanardTransferTypeRequest) {
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
        return true;
    }
    
    // GetNodeInfo Request (Service ID=1)
    if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID && transfer_type == CanardTransferTypeRequest) {
        printf("Accept GetNodeInfo Req\r\n"); // Debug
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    }

    // Dynamic ID Allocation (Message ID=1)
    // 注意：Dynamic Allocation ID 和 GetNodeInfo ID 都是 1，必须通过 transfer_type 区分
    if (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID && transfer_type == CanardTransferTypeBroadcast) {
        *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
        return true;
    }

    // Param GetSet
    if (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID && transfer_type == CanardTransferTypeRequest) {
        // printf("Accept Param Req\r\n"); // Debug
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
    }

    // Param ExecuteOpcode
    if (data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID && transfer_type == CanardTransferTypeRequest) {
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
        return true;
    }
    
    return false;
}

// 接收到完整传输后的回调函数
void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    // 每个完整传输计数一次
    stats_inc_transfer_rx();
    // 动态ID分配 (Message ID=1, Broadcast)
    if (transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID && 
        transfer->transfer_type == CanardTransferTypeBroadcast)
    {
        canardDynIDClientHandleAllocationResponse(&dynid_client, transfer);

        // If allocator echoed part of our UID but node_id is still 0, schedule quick follow-up chunk
        if (dyn_allocation_active && canardDynIDClientGetNodeID(&dynid_client) == 0 && dynid_client.uid_prefix_len < sizeof(node_unique_id)) {
            dyn_followup_pending = true;
            dyn_followup_deadline_ms = HAL_GetTick() + 10; // 10ms follow-up per spec window (<=400ms)
        } else {
            dyn_followup_pending = false;
        }
    }
    // GetNodeInfo Request (Service ID=1, Request)
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleGetNodeInfo(ins, transfer);
    }
    // Param GetSet Request
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleParamGetSet(ins, transfer);
    }
    // Param ExecuteOpcode Request
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleParamExecuteOpcode(ins, transfer);
    }
    // GetTransportStats Request (Service ID=4, Request)
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleGetTransportStats(ins, transfer);
    }
    // RestartNode Request (Service ID=5, Request)
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_RESTARTNODE_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleRestartNode(ins, transfer);
    }
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleBeginFirmwareUpdate(ins, transfer);
    }
    // 检查数据类型
    else if (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        // 解码接收到的数据
        // 注意：这里需要提供一个缓冲区给动态数组使用
        // libcanard 要求通过 dyn_arr_buf 参数传递缓冲区指针的地址
        static uavcan_equipment_actuator_Command cmd_buffer[15];
        uint8_t* p_dynamic_array_buffer = (uint8_t*)cmd_buffer;
        
        // 传入 &p_dynamic_array_buffer，这样解码函数内部会将 dest->commands.data 指向 cmd_buffer
        int32_t result = uavcan_equipment_actuator_ArrayCommand_decode(transfer, transfer->payload_len, &rx_actuator_cmd, &p_dynamic_array_buffer);
        
        if (result >= 0) {
            // 解码成功，处理命令
            processActuatorCommands(&rx_actuator_cmd);
        } else {
            printf("Decode fail: %d\r\n", result);
        }
    }
    else if (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
        // ESC RawCommand: int14 array, normalize to -1..1 then映射 PWM
        static int16_t raw_cmd_buf[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_CMD_MAX_LENGTH];
        uavcan_equipment_esc_RawCommand raw;
        raw.cmd.data = raw_cmd_buf;
        uint8_t* dyn_buf = (uint8_t*)raw_cmd_buf;
        int32_t result = uavcan_equipment_esc_RawCommand_decode(transfer, transfer->payload_len, &raw, &dyn_buf);

        if (result >= 0) {
            processEscRawCommand(&raw, transfer->source_node_id);
        } else {
            printf("ESC RawCommand decode fail: %d\r\n", result);
        }
    }
    
    // 释放传输负载内存
    canardReleaseRxTransferPayload(ins, transfer);
}

// 根据当前 TIM2 预分频和时钟，更新每微秒的计数刻度
static void update_tim2_tick_scale(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk = pclk1;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        tim_clk *= 2U; // APB1 分频不为1时，定时器时钟翻倍
    }
    tim2_ticks_per_us = (float)tim_clk / (float)(htim2.Init.Prescaler + 1U) / 1000000.0f;
}

static void start_dynamic_allocation(void) {
    dyn_allocation_active = true;
    dyn_followup_pending = false;
    dyn_last_req_ms = HAL_GetTick();
    canardDynIDClientInit(&dynid_client, node_unique_id);
    int req_res = canardDynIDClientStart(&dynid_client, &canard_ins, 0);
    if (req_res < 0) {
        printf("[DynID] start req err=%d\r\n", req_res);
    }
}

static bool wait_mailbox_result(uint32_t mailbox, uint32_t timeout_ms, bool* out_txok, uint32_t* out_tsr_snapshot)
{
    uint32_t rqcp_mask = 0U;
    uint32_t txok_mask = 0U;
    uint32_t abrq_mask = 0U;

    switch (mailbox) {
        case CAN_TX_MAILBOX0:
            rqcp_mask = CAN_TSR_RQCP0;
            txok_mask = CAN_TSR_TXOK0;
            abrq_mask = CAN_TSR_ABRQ0;
            break;
        case CAN_TX_MAILBOX1:
            rqcp_mask = CAN_TSR_RQCP1;
            txok_mask = CAN_TSR_TXOK1;
            abrq_mask = CAN_TSR_ABRQ1;
            break;
        case CAN_TX_MAILBOX2:
            rqcp_mask = CAN_TSR_RQCP2;
            txok_mask = CAN_TSR_TXOK2;
            abrq_mask = CAN_TSR_ABRQ2;
            break;
        default:
            return false;
    }

    uint32_t t0 = HAL_GetTick();
    while ((hcan.Instance->TSR & rqcp_mask) == 0U) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            hcan.Instance->TSR |= abrq_mask;
            if (out_tsr_snapshot != NULL) {
                *out_tsr_snapshot = hcan.Instance->TSR;
            }
            return false;
        }
    }

    uint32_t tsr = hcan.Instance->TSR;
    if (out_tsr_snapshot != NULL) {
        *out_tsr_snapshot = tsr;
    }
    if (out_txok != NULL) {
        *out_txok = ((tsr & txok_mask) != 0U);
    }

    // Clear request complete flag for this mailbox so next wait is valid.
    hcan.Instance->TSR |= rqcp_mask;
    return true;
}

// 处理发送队列
void processCanTxQueue(void) {

    const CanardCANFrame* tx_frame = canardPeekTxQueue(&canard_ins);
    
    while (tx_frame != NULL) {
        CAN_TxHeaderTypeDef tx_header = {
            .ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK,
            .IDE = CAN_ID_EXT,
            .RTR = CAN_RTR_DATA,
            .DLC = tx_frame->data_len
        };

        uint32_t mailbox;

        // 等待有空闲邮箱；若短暂等待后仍满，微延时让仲裁/邮箱释放，再尝试几次
        uint32_t wait_attempts = 0;
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0 && wait_attempts < 5) {
            HAL_Delay(1); // 1ms 让出 CPU，避免忙等
            wait_attempts++;
        }
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
            // 邮箱仍满，留到下一次循环继续发送
            return;
        }

        if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_frame->data, &mailbox) == HAL_OK) {
            bool txok = false;
            uint32_t tsr = 0U;
            bool complete = wait_mailbox_result(mailbox, 3U, &txok, &tsr);

            if (complete && txok) {
                // 仅在邮箱确认发送成功后再出队，避免“队列空但总线无ACK”的假象
                canardPopTxQueue(&canard_ins);
                stats_inc_frame_tx();
                stats_inc_transfer_tx();
            } else {
                uint32_t err = HAL_CAN_GetError(&hcan);
                printf("CAN Tx NotAck/Fail. MB:%lu Complete:%d TxOK:%d Err:0x%lX ESR:0x%08lX TSR:0x%08lX\r\n",
                       (unsigned long)mailbox,
                       complete ? 1 : 0,
                       txok ? 1 : 0,
                       (unsigned long)err,
                       (unsigned long)hcan.Instance->ESR,
                       (unsigned long)tsr);
                stats_inc_transfer_error();
                stats_inc_can_error();
                return;
            }
        } else {
            // 发送失败，打印错误原因
            uint32_t err = HAL_CAN_GetError(&hcan);
            uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
            printf("CAN Tx Fail. Err: 0x%X, FreeMB: %d, ESR: 0x%X\r\n", err, free_level, hcan.Instance->ESR);
            stats_inc_transfer_error();
            stats_inc_can_error();
            // 如果失败（含邮箱满/仲裁等），退出等待下一次主循环再尝试
            break;
        }
        
        tx_frame = canardPeekTxQueue(&canard_ins);
        if (tx_frame == NULL) {
            printf("[CAN TX] TX queue empty\r\n");
        }
    }
}

// 发送执行器命令的广播函数
int16_t sendActuatorCommand(CanardInstance* ins, uint8_t actuator_id, float value) {
    // 构建命令结构
    uavcan_equipment_actuator_Command cmd;
    cmd.actuator_id = actuator_id;
    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS;
    cmd.command_value = value;

    // 构建数组命令结构
    uavcan_equipment_actuator_ArrayCommand array_cmd;
    array_cmd.commands.len = 1;
    array_cmd.commands.data = &cmd;

    // 编码数据到buffer
    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
    uint32_t len = uavcan_equipment_actuator_ArrayCommand_encode(&array_cmd, buffer);

    // 发送广播
    return canardBroadcast(ins,
                           UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE,
                           UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                           &actuator_cmd_transfer_id,
                           CANARD_TRANSFER_PRIORITY_MEDIUM,
                           buffer,
                           len);
}

// 发送服务请求的示例
int16_t sendNodeStatusRequest(CanardInstance* ins, uint8_t target_node_id) {
    CanardTxTransfer transfer;
    canardInitTxTransfer(&transfer);
    
    static uint8_t node_status_transfer_id = 0;
    
    // 设置传输参数
    transfer.transfer_type = CanardTransferTypeRequest;
    transfer.data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
    transfer.data_type_id = UAVCAN_PROTOCOL_GETNODEINFO_ID;
    transfer.inout_transfer_id = &node_status_transfer_id;
    transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
    transfer.payload = NULL; // 空负载
    transfer.payload_len = 0;
    
    // 发送请求
    return canardRequestOrRespondObj(ins, target_node_id, &transfer);
}

void publishNodeStatus(void)
{
    uavcan_protocol_NodeStatus msg;
    msg.uptime_sec = HAL_GetTick() / 1000;
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.sub_mode = 0;
    msg.vendor_specific_status_code = 0;

    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint32_t len = uavcan_protocol_NodeStatus_encode(&msg, buffer);

    static uint8_t transfer_id = 0;
    int16_t res = canardBroadcast(&canard_ins, 
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
    if (res < 0) {
        printf("[NodeStatus] Broadcast failed: %d\r\n", res);
    }
}

// CAN接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanardCANFrame rx_frame;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        // 避免在中断中频繁打印，防止阻塞导致多帧丢失
        // printf("CAN RX ID: 0x%08X\r\n", rx_header.ExtId);

        can_rx_irq_count++;
        stats_inc_frame_rx();

        rx_frame.id = (rx_header.ExtId & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
        if (rx_header.RTR == CAN_RTR_REMOTE) {
            rx_frame.id |= CANARD_CAN_FRAME_RTR;
        }
        rx_frame.data_len = rx_header.DLC;
        memcpy(rx_frame.data, rx_data, rx_header.DLC);
        
        uint64_t timestamp_us = HAL_GetTick() * 1000;
        int16_t cr = canardHandleRxFrame(&canard_ins, &rx_frame, timestamp_us);
        if (cr < 0) {
            stats_inc_transfer_error();
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);
    printf("[CAN ERR] HAL:0x%08lX ESR:0x%08lX TSR:0x%08lX\r\n",
           (unsigned long)err,
           (unsigned long)hcan->Instance->ESR,
           (unsigned long)hcan->Instance->TSR);
    stats_inc_can_error();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
