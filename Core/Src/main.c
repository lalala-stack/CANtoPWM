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
#include "uavcan/protocol/NodeStatus.h"
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
void processCanRxFrames(void);
void processCanTxQueue(void);
int16_t sendActuatorCommand(CanardInstance* ins, uint8_t actuator_id, float value);
int16_t sendNodeStatusRequest(CanardInstance* ins, uint8_t target_node_id);
void publishNodeStatus(void);

// UAVCAN协议相关定义
#define UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE 0x07f0a7b40a594a47
#define UAVCAN_PROTOCOL_GETNODEINFO_ID 1
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

// --- 自检测试函数 ---
void run_parsing_test(void) {
    printf("\r\n=== Running Software Parsing Test ===\r\n");

    // 1. 临时设置一个本地节点ID，否则 shouldAcceptTransfer 会拒绝接收控制命令
    uint8_t original_node_id = canardGetLocalNodeID(&canard_ins);
    canardSetLocalNodeID(&canard_ins, 100); 

    // 2. 构造一个 ActuatorCommand 消息 (模拟上位机发送)
    uavcan_equipment_actuator_ArrayCommand cmd;
    uavcan_equipment_actuator_Command cmd_data[1]; // 创建一个静态数组来存储命令数据
    cmd.commands.len = 1;
    cmd.commands.data = cmd_data; // 将指针指向静态数组
    cmd.commands.data[0].actuator_id = 0;       // 控制第1个电机
    cmd.commands.data[0].command_type = 0;   // UNITLESS
    cmd.commands.data[0].command_value = 0.5f;  // 50% 油门 (预期脉宽 1750us)

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
    uint32_t len = uavcan_equipment_actuator_ArrayCommand_encode(&cmd, buffer);

    // 3. 添加 UAVCAN 传输层尾字节 (Tail Byte)
    // 单帧传输: Start=1, End=1, Toggle=0, TID=0 => 0xC0
    buffer[len] = 0xC0; 
    len++;

    // 4. 构造 CAN 帧
    CanardCANFrame frame;
    // ID结构: Priority(5) | MsgID(16) | SourceID(7)
    // Priority=Medium(16), MsgID=1010 (ArrayCommand), SourceID=42 (模拟发送者)
    frame.id = (CANARD_TRANSFER_PRIORITY_MEDIUM << 24) | 
               (UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID << 8) | 
               42;
    frame.id |= CANARD_CAN_FRAME_EFF; // 扩展帧标志
    frame.data_len = len;
    memcpy(frame.data, buffer, len);

    // 5. 注入到协议栈
    printf("Injecting fake CAN frame (ID: 0x%X, Len: %d)...\r\n", frame.id, frame.data_len);
    canardHandleRxFrame(&canard_ins, &frame, HAL_GetTick() * 1000);

    // 6. 恢复原来的节点ID状态
    canardSetLocalNodeID(&canard_ins, original_node_id);
    
    printf("=== Test Finished ===\r\n\r\n");
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

	// 初始化动态ID客户端
	canardDynIDClientInit(&dynid_client, node_unique_id);
	// 设置本地节点ID为匿名
	canardSetLocalNodeID(&canard_ins, CANARD_BROADCAST_NODE_ID);

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
	HAL_CAN_ConfigFilter(&hcan, &filter);

	// 启动CAN
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// 启动动态ID分配过程
	if (canardDynIDClientStart(&dynid_client, &canard_ins, 0) < 0)
	{
			// 错误处理
			printf("Failed to start dynamic ID allocation\r\n");
			Error_Handler();
	}

    // 运行自检测试
    //run_parsing_test();

    static uint32_t last_allocation_request_time = 0;
  while (1)
  {
		// 等待动态ID分配完成
		while(canardDynIDClientGetNodeID(&dynid_client) == CANARD_BROADCAST_NODE_ID)
		{
                // 每隔1秒重发一次分配请求
                if (HAL_GetTick() - last_allocation_request_time > 1000)
                {
                    int16_t res = canardDynIDClientStart(&dynid_client, &canard_ins, 0);
                    last_allocation_request_time = HAL_GetTick();
                    printf("Requesting ID... Res: %d\r\n", res);
                }

				processCanTxQueue(); // 持续处理发送队列

				HAL_Delay(10); // 短暂延迟
		}

		// ID分配成功，设置节点ID
		if (canardGetLocalNodeID(&canard_ins) == CANARD_BROADCAST_NODE_ID)
		{
				uint8_t allocated_id = canardDynIDClientGetNodeID(&dynid_client);
				canardSetLocalNodeID(&canard_ins, allocated_id);
				printf("Node ID allocated: %d\r\n", allocated_id);
		}

    // 处理发送队列
    processCanTxQueue();
    
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
// 决定是否接受某个传输的回调函数
bool shouldAcceptTransfer(const CanardInstance* ins,
                         uint64_t* out_data_type_signature,
                         uint16_t data_type_id,
                         CanardTransferType transfer_type,
                         uint8_t source_node_id) {
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
        return false;
    }

    // 根据数据类型ID决定是否接受
    if (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
        return true;
    }
    
    // 可以添加更多数据类型的判断
    // else if (data_type_id == ANOTHER_DATA_TYPE_ID) {
    //     *out_data_type_signature = ANOTHER_SIGNATURE;
    //     return true;
    // }
    
    return false;
}

// 接收到完整传输后的回调函数
void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    // 如果是动态ID分配的响应
    if (transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID)
    {
        canardDynIDClientHandleAllocationResponse(&dynid_client, transfer);
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

// 处理发送队列
void processCanTxQueue(void) {
    CanardCANFrame* tx_frame = canardPeekTxQueue(&canard_ins);
    
    while (tx_frame != NULL) {
        CAN_TxHeaderTypeDef tx_header = {
            .ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK,
            .IDE = CAN_ID_EXT,
            .RTR = CAN_RTR_DATA,
            .DLC = tx_frame->data_len
        };
        
        uint32_t mailbox;
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
    canardBroadcast(&canard_ins, 
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

// CAN接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanardCANFrame rx_frame;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        rx_frame.id = rx_header.ExtId;
        rx_frame.data_len = rx_header.DLC;
        memcpy(rx_frame.data, rx_data, rx_header.DLC);
        
        uint64_t timestamp_us = HAL_GetTick() * 1000;
        canardHandleRxFrame(&canard_ins, &rx_frame, timestamp_us);
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
