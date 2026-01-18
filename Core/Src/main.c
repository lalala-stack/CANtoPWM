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
#include <stdio.h>
#include "canard_dynamic_node_id_client.h"
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
void processActuatorCommands(uavcan_equipment_actuator_ArrayCommand* cmd);
void processEscRawCommand(uavcan_equipment_esc_RawCommand* raw);
void processCanRxFrames(void);
void processCanTxQueue(void);
int16_t sendActuatorCommand(CanardInstance* ins, uint8_t actuator_id, float value);
int16_t sendNodeStatusRequest(CanardInstance* ins, uint8_t target_node_id);
void publishNodeStatus(void);
void handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer);
void handleParamGetSet(CanardInstance* ins, CanardRxTransfer* transfer);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
typedef struct {
    const char* name;
    float value;
    float min;
    float max;
    float def;
    bool is_integer; // 简单的类型标记
} ParamEntry;

#define PARAM_COUNT 2
static ParamEntry params[PARAM_COUNT] = {
    {"can_node_id", 60.0f, 1.0f, 127.0f, 60.0f, true},
    {"actuator_id_offset", 0.0f, 0.0f, 10.0f, 0.0f, true}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void read_unique_id(uint8_t* out_uid);
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

// --- 动态ID分配测试函数 (修复版) ---
void run_dynamic_id_test(void) {
    printf("\r\n=== Running Dynamic ID Allocation Test ===\r\n");

    // 1. 创建一个临时的服务器实例
    CanardInstance server_ins;
    uint8_t server_mem[1024];
    canardInit(&server_ins, server_mem, sizeof(server_mem), NULL, NULL, NULL);
    canardSetLocalNodeID(&server_ins, 127); // 假设服务器 ID 为 127

    // 2. 构造 Allocation 消息 (模拟服务器给我们的响应)
    uavcan_protocol_dynamic_node_id_Allocation msg;
    msg.node_id = 42; // 服务器决定分配给我们的 ID
    msg.first_part_of_unique_id = 1; // 这是第一阶段（包含完整UID）
    msg.unique_id.len = 16;
    msg.unique_id.data = node_unique_id; // 必须匹配我们自己的 UID

    uint8_t buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
    uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, buffer);

    // 3. 让服务器广播这个消息
    static uint8_t transfer_id = 0;
    canardBroadcast(&server_ins, 
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buffer,
                    len);

    // 4. 注入帧 (libcanard 会自动处理多帧重组)
    printf("Injecting Allocation Response (Allocated ID: 42)...\r\n");
    inject_rx_frame_from_server(&server_ins, HAL_GetTick() * 1000);

    // 5. 检查结果
    uint8_t new_id = canardDynIDClientGetNodeID(&dynid_client);
    printf("Client State ID: %d\r\n", new_id);
    
    if (new_id == 42) {
        printf("SUCCESS: Dynamic ID Allocation Logic Verified!\r\n");
        // 模拟主循环：应用分配到的 ID
        canardSetLocalNodeID(&canard_ins, new_id);
    } else {
        printf("FAILURE: Client did not accept the ID.\r\n");
    }
    
    printf("=== Dynamic ID Test Finished ===\r\n\r\n");
}

// --- 电机控制测试函数 (修复版) ---
void inject_actuator_command(uint8_t actuator_id, float value) {
    // 创建临时服务器
    CanardInstance server_ins;
    uint8_t server_mem[512];
    canardInit(&server_ins, server_mem, sizeof(server_mem), NULL, NULL, NULL);
    canardSetLocalNodeID(&server_ins, 42); // 模拟发送者 ID

    uavcan_equipment_actuator_ArrayCommand cmd;
    uavcan_equipment_actuator_Command cmd_data[1];
    cmd.commands.len = 1;
    cmd.commands.data = cmd_data;
    cmd.commands.data[0].actuator_id = actuator_id;
    cmd.commands.data[0].command_type = 0;
    cmd.commands.data[0].command_value = value;

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
    uint32_t len = uavcan_equipment_actuator_ArrayCommand_encode(&cmd, buffer);

    static uint8_t transfer_id = 0;
    canardBroadcast(&server_ins, 
                    UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE,
                    UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buffer,
                    len);

    printf("Injecting CMD: ID=%d, Val=%.2f -> ", actuator_id, value);
    inject_rx_frame_from_server(&server_ins, HAL_GetTick() * 1000);
}

// --- 综合测试函数 ---
void run_parsing_test(void) {
    // 1. 先测试动态 ID 分配
    run_dynamic_id_test();

    // 2. 如果 ID 分配成功，测试电机控制
    if (canardGetLocalNodeID(&canard_ins) != CANARD_BROADCAST_NODE_ID) {
        printf("=== Running Motor Control Test ===\r\n");
        inject_actuator_command(0, -1.0f);
        inject_actuator_command(1, 1.0f);
        inject_actuator_command(2, 0.0f);
        inject_actuator_command(3, 0.5f);
        printf("=== Motor Test Finished ===\r\n");
    } else {
        printf("Skipping Motor Test (No ID assigned)\r\n");
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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

  // 启动TIM2的4个PWM通道
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // 初始化libcanard
	canardInit(&canard_ins,
						canard_memory_pool,
						sizeof(canard_memory_pool),
						onTransferReception,
						shouldAcceptTransfer,
          NULL);

  // 固定节点ID为 60
  canardSetLocalNodeID(&canard_ins, 60);

	// 初始化CAN过滤器
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
	
    // 调试：放宽过滤器，接收所有包
    filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
    
    HAL_CAN_ConfigFilter(&hcan, &filter);

	// 启动CAN
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 固定ID模式：直接处理发送队列
      processCanTxQueue();
    
    // 发送心跳包 (1Hz)
    static uint32_t last_node_status_time = 0;
    if (HAL_GetTick() - last_node_status_time >= 1000)
    {
        printf("My Node ID: %d\r\n", canardGetLocalNodeID(&canard_ins));
        publishNodeStatus();
        last_node_status_time = HAL_GetTick();
    }

    // 清理过期的传输
    uint64_t timestamp = HAL_GetTick() * 1000; // 转换为微秒
    canardCleanupStaleTransfers(&canard_ins, timestamp);
    
    HAL_Delay(1); // 短暂延迟
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(led_cnt >= 1000){
			HAL_GPIO_TogglePin( GPIOC , GPIO_PIN_13 );
			led_cnt = 0;
		}
		else{
			led_cnt++;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/* USER CODE BEGIN 4 */
// 处理GetNodeInfo请求
void handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer) {
    printf("[GetNodeInfo] Request Received from Node %d\r\n", transfer->source_node_id);

    uavcan_protocol_GetNodeInfoResponse resp;
    memset(&resp, 0, sizeof(resp));

    // Fill Hardware Version
    read_unique_id(resp.hardware_version.unique_id);
    resp.hardware_version.major = 1;
    resp.hardware_version.minor = 0;

    // Fill Software Version
    resp.software_version.major = 1;
    resp.software_version.minor = 0;
    resp.software_version.optional_field_flags = 0; 
    resp.software_version.vcs_commit = 0;

    // Fill Node Status
    resp.status.uptime_sec = HAL_GetTick() / 1000;
    resp.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    resp.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    
    // Set Node Name
    const char* name = "org.ardupilot.cantopwm";
    resp.name.len = strlen(name);
    resp.name.data = (uint8_t*)name;

    // 使用 static 避免 ISR 堆栈溢出
    static uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&resp, buffer);

    int16_t res = canardRequestOrRespond(ins,
                           transfer->source_node_id,
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
        // Debug: response enqueued, wait for processCanTxQueue to flush
        printf("[GetNodeInfo] Reply Enqueued, len=%lu\r\n", (unsigned long)len);
    }
}

// 处理Param.GetSet请求
void handleParamGetSet(CanardInstance* ins, CanardRxTransfer* transfer) {
     // use static to avoid stack overflow in ISR
     static uavcan_protocol_param_GetSetRequest req;
     static uint8_t name_buf[92]; 
     uint8_t* p_name_buf = name_buf;
     
     if (uavcan_protocol_param_GetSetRequest_decode(transfer, transfer->payload_len, &req, &p_name_buf) < 0) {
         printf("[Param] Decode Failed\r\n");
         return; 
     }

     if(req.name.len > 0) {
         printf("[Param] Request Name: %.*s\r\n", req.name.len, req.name.data);
     } else {
         printf("[Param] Request Index: %d\r\n", req.index);
     }

     ParamEntry* found = NULL;

     // 1. Try index if name is empty
     if (req.name.len == 0) {
        if (req.index < PARAM_COUNT) {
            found = &params[req.index];
        }
     } else {
         // 2. Try name
         for (int i=0; i<PARAM_COUNT; i++) {
             if (strncmp((const char*)req.name.data, params[i].name, req.name.len) == 0 && 
                 strlen(params[i].name) == req.name.len) {
                 found = &params[i];
                 break;
             }
         }
     }

     // If found and value is set (not empty), update it
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
         
         printf("[Param] Set %s = %f\r\n", found->name, found->value);

         // 如果修改了 can_node_id，立即应用到本地节点 ID
         if (strcmp(found->name, "can_node_id") == 0) {
             uint8_t new_id = (uint8_t)found->value;
             if (new_id >= CANARD_MIN_NODE_ID && new_id <= CANARD_MAX_NODE_ID) {
                 canardSetLocalNodeID(&canard_ins, new_id);
                 printf("[Param] Applied Node ID = %d\r\n", new_id);
             } else {
                 printf("[Param] Node ID out of range: %d\r\n", new_id);
             }
         }
     }

     // Prepare Response
     // use static to avoid stack overflow
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
     }

     static uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
     uint32_t len = uavcan_protocol_param_GetSetResponse_encode(&resp, buffer);

     int16_t res = canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
      if(res <= 0) printf("[Param] Reply Failed: %d\r\n", res);
}

// 决定是否接受某个传输的回调函数
bool shouldAcceptTransfer(const CanardInstance* ins,
                         uint64_t* out_data_type_signature,
                         uint16_t data_type_id,
                         CanardTransferType transfer_type,
                         uint8_t source_node_id) {
    
    // 只有在接收到关键服务请求时才打印，避免刷屏
    if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID || data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
         printf("Accept: ID=%d Type=%d Src=%d\r\n", data_type_id, transfer_type, source_node_id);
    }

        // Debug: 观测 RawCommand 是否进入过滤逻辑
        if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
            printf("Seen RawCommand ID=1030 Type=%d Src=%d\r\n", transfer_type, source_node_id);
        }

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
        printf("Accept RawCommand ID=1030 Type=%d Src=%d\r\n", transfer_type, source_node_id);
        *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
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
    if (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
        // printf("Accept Param Req\r\n"); // Debug
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
    }
    
    // Debug: 打印被拒绝的包信息，有助于通过排除法
    // if(data_type_id != 341) 
    printf("Reject: ID=%d Type=%d Src=%d\r\n", data_type_id, transfer_type, source_node_id);

    return false;
}

// 接收到完整传输后的回调函数
void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    // 动态ID分配 (Message ID=1, Broadcast)
    if (transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID && 
        transfer->transfer_type == CanardTransferTypeBroadcast)
    {
        canardDynIDClientHandleAllocationResponse(&dynid_client, transfer);
    }
    // GetNodeInfo Request (Service ID=1, Request)
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID &&
             transfer->transfer_type == CanardTransferTypeRequest) {
        handleGetNodeInfo(ins, transfer);
    }
    // Param GetSet Request
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
        handleParamGetSet(ins, transfer);
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
        printf("[ESC Raw] src=%d len=%d\r\n", transfer->source_node_id, transfer->payload_len);
        int32_t result = uavcan_equipment_esc_RawCommand_decode(transfer, transfer->payload_len, &raw, &dyn_buf);

        if (result >= 0) {
            printf("[ESC Raw] decode ok, cmd_len=%d\r\n", raw.cmd.len);
            processEscRawCommand(&raw);
        } else {
            printf("ESC RawCommand decode fail: %d\r\n", result);
        }
    }
    
    // 释放传输负载内存
    canardReleaseRxTransferPayload(ins, transfer);
}

// 处理执行器命令的实际函数
void processActuatorCommands(uavcan_equipment_actuator_ArrayCommand* cmd) {
    printf("Received ArrayCommand with %d commands:\r\n", cmd->commands.len);
    // 遍历所有接收到的命令
    for (uint8_t i = 0; i < cmd->commands.len; i++) {
        // 获取单个执行器命令
        uavcan_equipment_actuator_Command* single_cmd = &cmd->commands.data[i];
        
        // 将标准化的 command_value (-1.0 to 1.0) 转换为 PWM 脉宽 (1000 to 2000)
        // 1500 是中立点 (1.5ms), 500 是变化范围 (0.5ms)
        uint16_t pulse = 1500 + (uint16_t)(single_cmd->command_value * 500.0f);

        // 限制脉宽在安全范围内 (例如 1000-2000)
        if (pulse < 1000) pulse = 1000;
        if (pulse > 2000) pulse = 2000;

        printf("  - Actuator ID: %d, Value: %f, Pulse: %d us\r\n", 
               single_cmd->actuator_id, 
               single_cmd->command_value, 
               pulse);

        // 根据 actuator_id 更新对应的 PWM 通道
        // 假设 actuator_id 0-3 对应 TIM2_CH1-4
        switch (single_cmd->actuator_id) {
            case 0:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
                break;
            case 1:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
                break;
            case 2:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
                break;
            case 3:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
                break;
            default:
                // 其他 actuator_id 不处理
                break;
        }
    }
}

static int32_t get_actuator_offset(void) {
    // 使用参数表里的 actuator_id_offset，做四舍五入取整
    float v = params[1].value;
    return (int32_t)(v + (v >= 0 ? 0.5f : -0.5f));
}

// 处理 ESC RawCommand：将 int14[-8192,8191] 归一化到 -1..1 再映射 PWM
void processEscRawCommand(uavcan_equipment_esc_RawCommand* raw) {
    printf("Received ESC RawCommand with %d entries:\r\n", raw->cmd.len);

    const int32_t offset = get_actuator_offset();

    for (uint8_t i = 0; i < raw->cmd.len && i < 4; i++) {
        int16_t rc = raw->cmd.data[i];
        // 按规范 -8192..8191 归一化
        float norm = (float)rc / 8192.0f;
        if (norm < -1.0f) norm = -1.0f;
        if (norm > 1.0f) norm = 1.0f;

        uint16_t pulse = 1500 + (uint16_t)(norm * 500.0f);
        if (pulse < 1000) pulse = 1000;
        if (pulse > 2000) pulse = 2000;

        int32_t ch = (int32_t)i + offset;
        printf("  - ESC[%d]=>CH%ld rc=%d norm=%.3f pulse=%u us\r\n", i, (long)ch, rc, norm, pulse);

        if (ch < 0 || ch > 3) {
            continue; // 超出映射范围则忽略
        }

        switch (ch) {
            case 0:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
                break;
            case 1:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
                break;
            case 2:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
                break;
            case 3:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
                break;
            default:
                break;
        }
    }
}

// 处理发送队列
void processCanTxQueue(void) {
    // 强制打印，确保函数进来了 (调试完成后请删除)
    // static uint32_t loop_cnt = 0;
    // if(loop_cnt++ % 1000 == 0) printf("ProcTX Q: %p\r\n", canard_ins.tx_queue);

    const CanardCANFrame* tx_frame = canardPeekTxQueue(&canard_ins);
    
    while (tx_frame != NULL) {
        CAN_TxHeaderTypeDef tx_header = {
            .ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK,
            .IDE = CAN_ID_EXT,
            .RTR = CAN_RTR_DATA,
            .DLC = tx_frame->data_len
        };
        
        uint32_t mailbox;
        // Debug: 打印发送的 CAN ID 和长度
        printf("TX ID: 0x%08X DLC:%d\r\n", tx_header.ExtId, tx_header.DLC);
        // 若是服务响应帧，DataTypeID=1/目标节点等会体现在 tx_header.ExtId 中
        // 可根据需要展开 tx_frame->data 观察碎片
        
        // 尝试等待有空闲邮箱
        uint32_t retry = 0;
        while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0 && retry < 1000) { retry++; }

        if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_frame->data, &mailbox) == HAL_OK) {
            // 成功发送，从队列移除
            canardPopTxQueue(&canard_ins);
        } else {
            // 发送失败，打印错误原因
            uint32_t err = HAL_CAN_GetError(&hcan);
            uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
            printf("CAN Tx Fail. Err: 0x%X, FreeMB: %d, ESR: 0x%X\r\n", err, free_level, hcan.Instance->ESR);
            
            // 如果是邮箱满，就不需要一直打印了，直接退出等待下一次
            break;
        }
        
        tx_frame = canardPeekTxQueue(&canard_ins);
    }
}

// 发送执行器命令的广播函数
int16_t sendActuatorCommand(CanardInstance* ins, uint8_t actuator_id, float value) {
    CanardTxTransfer transfer;
    canardInitTxTransfer(&transfer);
    
    // 准备数据
    // 这里简化处理，实际应根据协议格式编码数据
    uint8_t buffer[8];
    // 假设将actuator_id和value编码到buffer中
    
    // 设置传输参数
    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
    transfer.data_type_id = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID;
    transfer.inout_transfer_id = &actuator_cmd_transfer_id;
    transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
    transfer.payload = buffer;
    transfer.payload_len = sizeof(buffer);
    
    // 发送广播
    return canardBroadcastObj(ins, &transfer);
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
    // printf("NodeStatus TX: %d\r\n", res);
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

        rx_frame.id = (rx_header.ExtId & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
        if (rx_header.RTR == CAN_RTR_REMOTE) {
            rx_frame.id |= CANARD_CAN_FRAME_RTR;
        }
        rx_frame.data_len = rx_header.DLC;
        memcpy(rx_frame.data, rx_data, rx_header.DLC);
        
        uint64_t timestamp_us = HAL_GetTick() * 1000;
        int16_t cr = canardHandleRxFrame(&canard_ins, &rx_frame, timestamp_us);
        if (cr < 0) {
            printf("canardHandleRxFrame err=%d\r\n", cr);
        }
    }
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
