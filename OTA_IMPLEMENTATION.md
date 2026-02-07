# DroneCAN OTA 实现说明

## 概述

本项目现已支持通过 DroneCAN GUI Tool 进行 OTA (Over-The-Air) 固件更新。

## 实现的组件

### 1. 应用程序固件 (Application Firmware)

**文件位置：** `Core/Src/main.c`, `Core/Src/app_logic.c`

**已实现功能：**
- ✅ `BeginFirmwareUpdate` 服务处理 (Service ID 40)
- ✅ Bootloader 标志设置 (使用 BKP->DR1 备份寄存器)
- ✅ 优雅重启（等待 CAN 发送队列清空）
- ✅ 应用程序运行时 LED 快闪 6 次指示

**关键代码位置：**
- `main.c:70-71` - Bootloader 标志定义
- `main.c:153-159` - 设置 Bootloader 标志
- `main.c:161-193` - BeginFirmwareUpdate 处理
- `main.c:357-366` - 重启流程
- `main.c:476-478` - 应接受 BeginFirmwareUpdate 服务
- `main.c:551-553` - BeginFirmwareUpdate 路由

### 2. Bootloader 固件

**文件位置：**
- `Core/Src/bootloader.c` - Bootloader 主逻辑
- `Core/Inc/bootloader.h` - Bootloader 头文件
- `Core/Src/bootloader_can.c` - DroneCAN CAN 协议实现
- `Core/Inc/bootloader_can.h` - CAN 协议头文件

**已实现功能：**
- ✅ 启动时检查 Bootloader 标志
- ✅ 应用程序有效性检查（栈指针和复位向量验证）
- ✅ Flash 擦除和写入
- ✅ 跳转到应用程序
- ✅ DroneCAN GetNodeInfo 服务
- ✅ DroneCAN File Read 服务（基础框架）
- ✅ NodeStatus 发布（模式：SOFTWARE_UPDATE）
- ⚠️ 完整文件读取流程（需要进一步实现）

## 使用流程

### 通过 DroneCAN GUI Tool 更新固件

1. **准备工作**
   - 确保设备上已运行应用程序固件
   - 连接 CAN 总线和电源
   - 启动 DroneCAN GUI Tool

2. **发送 BeginFirmwareUpdate 请求**
   - 在 GUI Tool 中选择节点
   - 执行 BeginFirmwareUpdate 服务
   - 指定源节点和固件路径

3. **应用程序响应**
   - 应用程序收到请求后回复 OK
   - 设置 BKP->DR1 标志为 0xB007B007
   - 等待 CAN 发送队列清空
   - 执行 NVIC_SystemReset()

4. **Bootloader 启动**
   - 复位后检测到 BKP->DR1 标志
   - 进入 Bootloader 模式
   - LED 保持常亮指示 Bootloader 模式

5. **固件传输**
   - Bootloader 接收固件数据
   - 擦除 Flash 应用程序区域
   - 将固件写入 Flash
   - 验证固件有效性

6. **启动新固件**
   - 清除 BKP->DR1 标志
   - LED 慢闪 3 次指示成功
   - 跳转到新固件
   - 应用程序启动时 LED 快闪 6 次

## 内存布局

### STM32F103CBT6 (128KB Flash)

```
0x08000000  ┌─────────────────────────┐
            │   Application Flash     │
            │   (约 124KB)            │
            │                         │
            │                         │
0x0801F800  ├─────────────────────────┤
            │   Bootloader (可选)     │  如果需要独立 Bootloader
            │   (2KB)                  │
0x0801FC00  ├─────────────────────────┤
            │   Parameters            │  参数存储（1页）
            │   (1KB)                 │
0x0801FFFF  └─────────────────────────┘
```

当前实现使用内联 Bootloader（与应用程序在同一固件中），
启动时检查标志决定运行模式。

## 硬件要求

### 备份寄存器配置

STM32F103CBT6 的 BKP 寄存器需要：
- 启用 BKP 时钟：`__HAL_RCC_BKP_CLK_ENABLE()`
- 启用后备域访问：`HAL_PWR_EnableBkUpAccess()`

## 待完善的功能

### 1. 完整的文件读取协议

当前 `bootloader_can.c` 中的 `handleFileRead` 只是一个框架。
需要实现：
- 接收固件文件数据块
- 写入 Flash
- 跟踪写入进度
- 处理文件读取错误
- 超时处理

### 2. 固件验证

添加：
- CRC32 校验
- 签名验证（可选）
- 版本兼容性检查

### 3. 错误恢复

- 传输失败时回退到旧固件
- 多次重试机制
- 错误代码和状态指示

### 4. 进度反馈

- 通过 NodeStatus 报告更新进度
- 支持 QueryProgress 服务（可选）

## 编译和部署

### 1. 添加新文件到项目

将以下文件添加到 Keil MDK-ARM 项目：
- `Core/Src/bootloader.c`
- `Core/Inc/bootloader.h`
- `Core/Src/bootloader_can.c`
- `Core/Inc/bootloader_can.h`

### 2. 编译项目

在 Keil MDK-ARM 中编译项目，确保没有错误。

### 3. 烧录固件

烧录生成的 `.hex` 或 `.axf` 文件到 STM32F103CBT6。

### 4. 测试 OTA

1. 运行应用程序，验证正常运行
2. 使用 DroneCAN GUI Tool 发送 BeginFirmwareUpdate
3. 观察复位行为
4. 确认新固件成功启动

## 故障排查

### 设备无法复位到 Bootloader

- 检查 BKP->DR1 标志是否正确设置
- 确认备份域时钟已启用
- 验证 BKP 寄存器访问权限

### 新固件无法启动

- 验证 Flash 写入是否成功
- 检查栈指针和复位向量地址
- 使用调试器检查复位向量地址

### CAN 通信失败

- 确认 CAN 总线终端电阻
- 检查波特率配置（当前：1Mbps）
- 验证 CAN 过滤器配置

## 注意事项

1. **备份寄存器依赖：** 使用 BKP->DR1 标志需要后备域供电（VBAT）
   - 如果 VBAT 未连接，复位后标志会丢失
   - 替代方案：使用 Flash 特定地址存储标志

2. **Flash 写入保护：** 确保 Flash 写入保护已禁用
   - 检查 FLASH_WRP 寄存器

3. **看门狗：** 如果启用了看门狗，在 OTA 过程中需要定期喂狗

4. **固件大小：** 确保新固件不超过应用程序区域大小

## 参考资料

- [DroneCAN Specification](https://dronecan.org/specification)
- [STM32F103 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [libcanard Library](https://github.com/107-systems/libcanard)
