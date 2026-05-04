# Dual_Closed-LOOP_Driver

双路闭环电机驱动器，基于 **STM32F103C8T6**，支持速度/位置双模式闭环控制，集成 **VOFA 上位机** 实时波形显示与调参功能。

> **当前为 VOFA-only 模式**：上电后自动以 200 Hz 输出 JustFloat 波形，调参通过 FireWater 文本命令完成。调试完成后可一键恢复完整二进制协议。

---

## 目录

- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [通信协议](#通信协议)
- [快速开始（VOFA 调参）](#快速开始vofa-调参)
- [构建与烧录](#构建与烧录)
- [恢复二进制协议](#恢复二进制协议)
- [项目结构](#项目结构)
- [默认参数](#默认参数)

---

## 硬件平台

| 参数 | 值 |
|------|-----|
| MCU | STM32F103C8T6 (ARM Cortex-M3, 72 MHz) |
| Flash / RAM | 64 KB / 20 KB |

### 电机与引脚分配

| 电机 | 编码器 | PWM | 方向 GPIO |
|------|--------|-----|-----------|
| **电机 1** | TIM2 (PA0/PA1) | TIM4 CH1 (PB6) | PB7 (高电平=正转) |
| **电机 2** | TIM1 (PA8/PA9) | TIM3 CH1 (PA6) | PA5 (高电平=正转) |

### 其他外设

| 外设 | 引脚 | 用途 |
|------|------|------|
| USART2 | PA2 (TX) / PA3 (RX) | 上位机通信（VOFA / 自定义上位机） |
| I2C1 | PB8 (SCL) / PB9 (SDA) | 预留（12864 OLED 等） |

### 编码器参数

- 编码器线数：**1024**
- 解码模式：TI12 **四倍频**
- 传动比：电机转 1 圈，编码器转 **2 圈**
- **电机 1 圈 = 8192 个 TIM 计数**

```
角度(rad) = encoder_total × 2π / 8192
速度(rad/s) = delta_count × 2π × 1000 / 8192   (控制周期 1 ms)
```

---

## 软件架构

采用**模块化 + 非阻塞**设计，1 kHz 控制周期，所有模块均不阻塞：

| 模块 | 文件 | 职责 |
|------|------|------|
| **Encoder** | `encoder.c/h` | 编码器读取、16 位溢出补偿、速度/角度计算 |
| **Motor** | `motor.c/h` | PWM 输出（TIM3/TIM4）、方向控制、使能/急停 |
| **PID** | `pid.c/h` | 位置式 PID，积分限幅 + 抗积分饱和 |
| **Controller** | `controller.c/h` | 状态机、梯形轨迹规划、速度/位置串级闭环 |
| **Protocol** | `protocol.c/h` | UART DMA 非阻塞通信、协议解析/组帧 |
| **App** | `app.c/h` | 系统初始化、命令分发、状态回传调度 |

### 控制时序

- **SysTick 1 ms** → `App_ControlUpdate()`
  - 编码器更新（×2）
  - PID 计算 + 轨迹规划（×2）
  - PWM 输出更新（×2）
  - UART 命令轮询与解析
  - 状态/波形发送

### PWM 频率

TIM3/TIM4 的 ARR 在启动时被动态修改为 **1799**，得到：

```
36 MHz / 1800 = 20 kHz
```

避免电机低频啸叫，同时保留 1800 级分辨率。

---

## 通信协议

本项目支持**双协议切换**，通过 `protocol.h` 中的宏控制：

| 模式 | 下发指令 | 回传波形 | 适用场景 |
|------|----------|----------|----------|
| **VOFA-only**（默认） | FireWater 文本命令 | JustFloat | 调参、波形观察 |
| **Binary**（可恢复） | 二进制帧协议 | 标准 STATUS 帧 | 自定义上位机、高可靠通信 |

### VOFA-only 模式（当前默认）

上电即自动以 **200 Hz** 发送 JustFloat 帧，无需额外配置。

#### JustFloat 通道定义

| 通道 | 数据 | 单位 |
|------|------|------|
| ch0 | 电机1 实际速度 | rad/s |
| ch1 | 电机1 实际角度 | rad |
| ch2 | 电机1 PWM | — |
| ch3 | 电机1 轨迹目标速度 | rad/s |
| ch4 | 电机1 轨迹目标角度 | rad |
| ch5 | 电机2 实际速度 | rad/s |
| ch6 | 电机2 实际角度 | rad |
| ch7 | 电机2 PWM | — |
| ch8 | 电机2 轨迹目标速度 | rad/s |
| ch9 | 电机2 轨迹目标角度 | rad |

帧尾：`0x00 0x00 0x80 0x7f`

#### FireWater 文本命令

所有命令为 ASCII 文本，以 `\n` 结尾，可直接在 VOFA「命令」窗口输入：

| 命令 | 格式 | 示例 |
|------|------|------|
| 设置目标 | `M motor mode speed angle accel decel` | `M 0 0 10.0 0.0 20.0 20.0` |
| 设置 PID | `P motor pid_type kp ki kd` | `P 0 0 2.0 0.5 0.0` |
| 控制指令 | `C motor cmd` | `C 0 0`（使能）、`C 0 3`（急停） |
| 波形频率 | `V interval_ms` | `V 5`（200Hz）、`V 0`（关闭） |

参数说明：
- `motor`：`0`=电机1, `1`=电机2
- `mode`：`0`=速度模式, `1`=位置模式
- `pid_type`：`0`=速度环, `1`=位置环
- `cmd`：`0`=使能, `1`=失能, `2`=回零, `3`=急停, `4`=清除故障

> 详细协议文档见 [`COMM_PROTOCOL.md`](COMM_PROTOCOL.md)。

---

## 快速开始（VOFA 调参）

### 1. 硬件连接

- USB 转 TTL 连接 **PA2(TX)** 和 **PA3(RX)**，波特率 **115200**。
- 电机驱动器接 **PB6/PB7**（电机1）和 **PA6/PA5**（电机2）。
- 编码器接 **PA0/PA1**（电机1）和 **PA8/PA9**（电机2）。

### 2. VOFA 配置

1. 打开 **VOFA+**，选择串口，波特率 **115200-8-N1**。
2. 新建 **JustFloat** 控件，添加 **10 个通道**，按上表绑定。
3. 打开「命令」窗口，准备输入文本指令。

### 3. 上电调试流程

```
C 0 0                        (使能电机1)
M 0 0 5.0 0.0 10.0 10.0      (速度模式, 目标5 rad/s, 加减速10)
```

观察 ch0（实际速度）与 ch3（目标速度）的跟随情况。

**调 PID**：
```
P 0 0 3.0 0.8 0.0            (调速度环 Kp=3 Ki=0.8)
```

**位置模式**：
```
M 0 1 2.0 3.14 5.0 5.0       (位置模式, 目标π rad, 限速2, 加减速5)
```

**急停**：
```
C 0 3
```

**清除故障并重新使能**：
```
C 0 4
C 0 0
```

---

## 构建与烧录

### 环境要求

- GNU ARM Embedded Toolchain (`arm-none-eabi-gcc`)
- CMake ≥ 3.22
- Ninja

### 编译

```bash
cmake --preset Debug
cmake --build --preset Debug
```

### 烧录

**ST-Link GDB**（VS Code 直接调试）：
使用项目自带的 `.vscode/launch.json`。

**OpenOCD**：
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program build/Debug/Dual_Closed-LOOP_Driver.elf verify reset exit"
```

---

## 恢复二进制协议

调参完成后，如需对接自定义上位机，恢复完整二进制帧协议：

1. 打开 `Core/Inc/protocol.h`
2. 修改第 23 行：
   ```c
   #define PROTOCOL_VOFA_ONLY      0   /* 将 1 改为 0 */
   ```
3. 重新编译烧录

恢复后将启用：
- 二进制帧收发（`0xAA 0x55` 帧头，带校验和）
- 标准状态回传帧（`0x81`）
- 心跳应答（`0x85`）
- 同时保留 VOFA JustFloat 和文本命令支持

> 二进制协议完整定义见 [`COMM_PROTOCOL.md`](COMM_PROTOCOL.md)。

---

## 项目结构

```
.
├── Core/
│   ├── Inc/                    # 头文件
│   │   ├── app.h
│   │   ├── controller.h
│   │   ├── encoder.h
│   │   ├── motor.h
│   │   ├── pid.h
│   │   ├── protocol.h          # 协议切换宏 PROTOCOL_VOFA_ONLY 在此
│   │   └── main.h
│   └── Src/                    # 源文件
│       ├── app.c               # 主调度、命令分发
│       ├── controller.c        # 轨迹规划 + 串级 PID
│       ├── encoder.c           # 编码器读取 + 溢出处理
│       ├── motor.c             # PWM + 方向控制
│       ├── pid.c               # PID 算法
│       ├── protocol.c          # DMA UART + 协议解析
│       └── main.c              # HAL 初始化入口
├── Drivers/                    # STM32 HAL / CMSIS
├── cmake/
├── CMakeLists.txt
├── COMM_PROTOCOL.md            # 通信协议详细文档
└── README.md                   # 本文件
```

---

## 默认参数

| 参数 | 值 |
|------|-----|
| 控制周期 | 1 ms (1 kHz) |
| PWM 频率 | 20 kHz |
| VOFA JustFloat | 200 Hz（上电即输出） |
| 速度环 PID | Kp=2.0, Ki=0.5, Kd=0.0 |
| 位置环 PID | Kp=5.0, Ki=0.0, Kd=0.1 |
| 默认加速度 | 10 rad/s² |
| 默认减速度 | 10 rad/s² |
| PWM 输出范围 | ±1000（对应 55% 占空比，约 11 V / 20 V） |

---

## 注意事项

1. **USER CODE 保护**：`main.c`、`stm32f1xx_it.c`、`stm32f1xx_hal_msp.c` 中所有自定义代码均置于 `USER CODE BEGIN/END` 块内，重新生成 CubeMX 代码时不会被覆盖。
2. **不要修改 HAL 源码**：`Drivers/STM32F1xx_HAL_Driver/` 为生成代码，如需修改配置请调整 `.ioc` 文件后重新生成。
3. **编码器溢出**：控制周期 1 ms 内电机不可能溢出 16 位计数器（理论极限约 4000 转/秒），溢出处理仅作为极端工况保险。
4. **VOFA 与二进制共存**：`PROTOCOL_VOFA_ONLY = 0` 时，系统同时接受二进制帧和 FireWater 文本命令，发送端 JustFloat 和标准 STATUS 帧独立运行，冲突时自动跳过。
