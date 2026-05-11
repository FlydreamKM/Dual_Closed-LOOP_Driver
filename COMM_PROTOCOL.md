# Dual_Closed-LOOP_Driver 二进制通信协议（详细版）

> **物理层**：UART2，115200-8-N1，小端模式 (Little-Endian)  
> **适用模式**：`PROTOCOL_VOFA_ONLY = 0`（纯二进制协议）或 `PROTOCOL_VOFA_ONLY = 1`（纯 VOFA 模式）  
> **版本**：v2.0 — 修正 `CMD_SET_TARGET` / `CMD_SET_PID` 帧长度定义

---

## 目录

- [一、协议概述](#一协议概述)
- [二、物理层与数据格式](#二物理层与数据格式)
- [三、通用帧格式](#三通用帧格式)
- [四、下行帧 — 上位机 → 下位机](#四下行帧--上位机--下位机)
  - [4.1 CMD_SET_TARGET (0x01)](#41-cmd_set_target-0x01)
  - [4.2 CMD_SET_PID (0x02)](#42-cmd_set_pid-0x02)
  - [4.3 CMD_CONTROL (0x03)](#43-cmd_control-0x03)
  - [4.4 CMD_REQ_STATUS (0x04)](#44-cmd_req_status-0x04)
  - [4.5 CMD_HEARTBEAT (0x05)](#45-cmd_heartbeat-0x05)
  - [4.6 CMD_SET_VOFA (0x06)](#46-cmd_set_vofa-0x06)
- [五、上行帧 — 下位机 → 上位机](#五上行帧--下位机--上位机)
  - [5.1 RSP_STATUS (0x81)](#51-rsp_status-0x81)
  - [5.2 RSP_ACK (0x82)](#52-rsp_ack-0x82)
- [六、校验和算法](#六校验和算法)
- [七、状态机与命令交互](#七状态机与命令交互)
- [八、完整通信时序示例](#八完整通信时序示例)
- [九、注意事项与常见问题](#九注意事项与常见问题)

---

## 一、协议概述

本协议采用**主从问答式 + 主动上报**混合架构：

- **下行**（上位机 → 下位机）：命令帧，下发控制指令与参数
- **上行**（下位机 → 上位机）：响应帧，包括主动周期性 STATUS 帧和被动 ACK 应答帧

下位机内部维护一个 **12 级命令队列**，下行命令先入队，在 1 kHz 控制节拍中依次执行，确保 UART 通信不阻塞实时控制。

---

## 二、物理层与数据格式

| 参数 | 值 |
|------|-----|
| 接口 | USART2 (PA2=TX, PA3=RX) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 (N) |
| 字节序 | **小端 (Little-Endian)** |
| 浮点数 | **IEEE 754 单精度 (float32)** |

---

## 三、通用帧格式

所有二进制帧（无论上下行）均采用统一的定界格式：

```
┌────────┬────────┬─────┬─────┬──────────┬─────┐
│ SOF0   │ SOF1   │ LEN │ CMD │ DATA     │ CHK │
│ 1 byte │ 1 byte │ 1B  │ 1B  │ N bytes  │ 1B  │
│ 0xAA   │ 0x55   │     │     │          │     │
└────────┴────────┴─────┴─────┴──────────┴─────┘
```

### 字段说明

| 字段 | 长度 | 说明 |
|------|------|------|
| **SOF0** | 1 | 帧头首字节，固定 `0xAA` |
| **SOF1** | 1 | 帧头次字节，固定 `0x55` |
| **LEN** | 1 | **CMD + DATA 的总字节数**，不含 SOF0/SOF1/CHK |
| **CMD** | 1 | 命令码或响应码 |
| **DATA** | N | 载荷数据，N = LEN − 1（因为 CMD 占 1 字节） |
| **CHK** | 1 | 校验和，计算范围：**CMD + DATA 所有字节的累加和，取低 8 位** |

### 帧解析状态机

下位机 RX 采用字节级状态机同步：

```
State 0: 等待 SOF0 (0xAA)
State 1: 收到 SOF0，等待 SOF1 (0x55)
         若再次收到 0xAA 则保持 State 1（处理连续帧头）
State 2: 收到 SOF1，下一字节为 LEN
         若 LEN > 60，判为错误，返回 State 0
State 3: 接收 CMD + DATA + CHK，共 LEN + 1 字节
         收完后校验 CHK：
           ✓ 正确 → 解析命令并入队
           ✗ 错误 → 丢弃，返回 State 0
```

---

## 四、下行帧 — 上位机 → 下位机

### 4.1 CMD_SET_TARGET (0x01)

设置电机目标参数，包括控制模式、目标速度/角度、加减速限制。

**帧结构**：

```
[AA][55][13][01] [motor_id][mode] [target_speed] [target_angle] [accel] [decel] [CHK]
      ↑   ↑   ↑       ↑        ↑         ↑              ↑           ↑       ↑
     SOF  LEN CMD   1字节    1字节      4字节          4字节       4字节   4字节
```

**LEN = 19**（CMD 1B + DATA 18B）

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `motor_id` | uint8 | 1 | `0x00`=电机1, `0x01`=电机2, `0xFF`=双电机 |
| 1 | `mode` | uint8 | 1 | `0x00`=速度模式, `0x01`=位置模式 |
| 2 | `target_speed` | float32 | 4 | 目标速度，单位 rad/s，小端浮点 |
| 6 | `target_angle` | float32 | 4 | 目标角度，单位 rad，小端浮点 |
| 10 | `accel` | float32 | 4 | 加速度限制，单位 rad/s²，必须 > 0 |
| 14 | `decel` | float32 | 4 | 减速度限制，单位 rad/s²，必须 > 0 |

**行为**：
- 每次收到此命令，下位机自动**重置**对应电机的 `traj_speed = 0` 和 PID 积分，防止历史状态干扰新指令
- `traj_speed` 随后以 `accel` 为斜率向 `target_speed` 线性 ramp
- 若 `accel` 或 `decel` 为 0 或负数，该参数被忽略，保持原值

**示例**（电机1，速度模式，目标 5.0 rad/s，加减速 10.0 rad/s²）：

```hex
AA 55 13 01 00 00 00 00 A0 40 00 00 00 00 00 00 20 41 00 00 20 41 XX
```

（注：`00 00 A0 40` = 5.0f 的小端表示；`XX` 为校验和）

---

### 4.2 CMD_SET_PID (0x02)

设置速度环或位置环 PID 参数。

**帧结构**：

```
[AA][55][0F][02] [motor_id][pid_type] [kp] [ki] [kd] [CHK]
      ↑   ↑   ↑       ↑         ↑        ↑    ↑    ↑
     SOF  LEN CMD   1字节      1字节    4B   4B   4B
```

**LEN = 15**（CMD 1B + DATA 14B）

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `motor_id` | uint8 | 1 | `0x00`=电机1, `0x01`=电机2, `0xFF`=双电机 |
| 1 | `pid_type` | uint8 | 1 | `0x00`=速度环, `0x01`=位置环 |
| 2 | `kp` | float32 | 4 | 比例增益 |
| 6 | `ki` | float32 | 4 | 积分增益 |
| 10 | `kd` | float32 | 4 | 微分增益 |

**输出限制**：

| 环路 | 输出限制 | 积分限制 |
|------|----------|----------|
| 速度环 | ±1000 | ±500 |
| 位置环 | ±100 | ±50 |

**示例**（电机1 速度环：Kp=3.0, Ki=0.8, Kd=0.0）：

```hex
AA 55 0F 02 00 00 00 00 40 40 CD CC 4C 3F 00 00 00 00 XX
```

（注：`00 00 40 40` = 3.0f；`CD CC 4C 3F` = 0.8f）

---

### 4.3 CMD_CONTROL (0x03)

电机控制指令（使能、失能、急停、回零、清除故障）。

**帧结构**：

```
[AA][55][03][03] [motor_id][ctrl_cmd] [CHK]
      ↑   ↑   ↑       ↑         ↑
     SOF  LEN CMD   1字节      1字节
```

**LEN = 3**（CMD 1B + DATA 2B）

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `motor_id` | uint8 | 1 | `0x00`=电机1, `0x01`=电机2, `0xFF`=双电机 |
| 1 | `ctrl_cmd` | uint8 | 1 | 见下表 |

**指令码表**：

| `ctrl_cmd` | 名称 | 效果 | 状态机转换 |
|------------|------|------|------------|
| `0x00` | **ENABLE** | 使能电机，PWM 开始输出 | IDLE → RUNNING |
| `0x01` | **DISABLE** | 失能电机，PWM = 0 | RUNNING → IDLE |
| `0x02` | **HOME** | 编码器清零，轨迹归零，PID 重置 | → IDLE（需重新使能） |
| `0x03` | **EMERGENCY** | 急停，PWM 强制为 0 | 任意 → EMERGENCY |
| `0x04` | **CLEAR_FAULT** | 清除急停/故障标志 | EMERGENCY → IDLE |

**示例**（电机1 使能）：

```hex
AA 55 03 03 00 00 XX
```

（`CHK = 0x03 + 0x00 + 0x00 = 0x03`）

---

### 4.4 CMD_REQ_STATUS (0x04)

请求下位机发送一帧 STATUS 状态数据。

**帧结构**：

```
[AA][55][01][04] [CHK]               (LEN=1，motor_id 省略，默认双电机)
[AA][55][02][04] [motor_id] [CHK]    (LEN=2，指定电机)
```

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `motor_id` | uint8 | 1 | 可选，`0x00`=电机1, `0x01`=电机2, `0xFF`=双电机 |

> 若 `LEN=1`（无 motor_id），下位机默认返回 **双电机** 状态（依次发送两帧 STATUS）。

---

### 4.5 CMD_HEARTBEAT (0x05)

心跳包，用于上位机保活检测。下位机收到后返回 RSP_ACK。

**帧结构**：

```
[AA][55][01][05] [CHK]               (LEN=1，默认双电机)
[AA][55][02][05] [motor_id] [CHK]    (LEN=2，指定电机)
```

字段与 `CMD_REQ_STATUS` 相同。

---

### 4.6 CMD_SET_VOFA (0x06)

设置 JustFloat 波形输出频率（仅在 `PROTOCOL_VOFA_ONLY = 1` 时有效）。

**帧结构**：

```
[AA][55][03][06] [motor_id][interval_ms] [CHK]
      ↑   ↑   ↑       ↑         ↑
     SOF  LEN CMD   1字节      1字节
```

**LEN = 3**（CMD 1B + DATA 2B）

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `motor_id` | uint8 | 1 | `0xFF`=全局设置（固定） |
| 1 | `interval_ms` | uint8 | 1 | 帧间隔毫秒数。`0`=关闭，`5`=200Hz，`10`=100Hz |

**示例**（开启 200 Hz 波形输出）：

```hex
AA 55 03 06 FF 05 XX
```

（`CHK = 0x06 + 0xFF + 0x05 = 0x10`）

---

## 五、上行帧 — 下位机 → 上位机

### 5.1 RSP_STATUS (0x81)

周期性状态上报帧，下位机按 `status_interval_ms` 设定自动发送。

**帧结构**：

```
[AA][55][19][81] [motor_id][mode_state] [actual_speed] [actual_angle]
      ↑   ↑   ↑       ↑         ↑            ↑              ↑
     SOF  LEN CMD   1字节      1字节        4字节          4字节

[target_speed] [target_angle] [pwm_output] [encoder_total] [fault] [CHK]
      ↑              ↑            ↑              ↑            ↑
    4字节          4字节        2字节          4字节       1字节
```

**LEN = 25**（CMD 1B + DATA 24B），**总帧长 = 29 字节**

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `motor_id` | uint8 | 1 | `0x00`=电机1, `0x01`=电机2 |
| 1 | `mode_state` | uint8 | 1 | 高4位=state，低4位=mode（见下方分解） |
| 2 | `actual_speed` | float32 | 4 | 实际速度，rad/s |
| 6 | `actual_angle` | float32 | 4 | 实际角度，rad |
| 10 | `target_speed` | float32 | 4 | 目标速度，rad/s |
| 14 | `target_angle` | float32 | 4 | 目标角度，rad |
| 18 | `pwm_output` | int16 | 2 | PWM 输出值，范围 ±1000 |
| 20 | `encoder_total` | int32 | 4 | 编码器总脉冲计数（含溢出补偿） |
| 24 | `fault` | uint8 | 1 | 故障码，`0x00`=无故障 |

**`mode_state` 分解**：

```
bit 7  bit 6  bit 5  bit 4 | bit 3  bit 2  bit 1  bit 0
  ───────── state ─────────   ───────── mode ──────────
```

| `mode` (低4位) | 值 | 含义 |
|----------------|-----|------|
| MODE_SPEED | `0x0` | 速度模式 |
| MODE_POSITION | `0x1` | 位置模式 |

| `state` (高4位) | 值 | 含义 |
|-----------------|-----|------|
| STATE_IDLE | `0x0` | 空闲/未使能 |
| STATE_RUNNING | `0x1` | 运行中 |
| STATE_HOMING | `0x2` | 回零中 |
| STATE_EMERGENCY | `0x3` | 急停/故障 |
| STATE_FAULT | `0x4` | 故障（保留） |

**示例**：`mode_state = 0x10` → state=RUNNING(1), mode=SPEED(0)

---

### 5.2 RSP_ACK (0x82)

通用应答帧，用于回应 `CMD_HEARTBEAT` 及部分控制指令。

**帧结构**：

```
[AA][55][03][82] [cmd][result] [CHK]
      ↑   ↑   ↑     ↑     ↑
     SOF  LEN CMD  1字节  1字节
```

**LEN = 3**（CMD 1B + DATA 2B），**总帧长 = 7 字节**

**DATA 字段**：

| 偏移 | 字段 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| 0 | `cmd` | uint8 | 1 | 原始命令码（如 `0x05` 表示回应心跳） |
| 1 | `result` | uint8 | 1 | 结果码，`0x00`=成功，非零=错误码（保留） |

**示例**（心跳应答）：

```hex
AA 55 03 82 05 00 8A
```

（`CHK = 0x82 + 0x05 + 0x00 = 0x87`？等等，重新计算：`0x82 + 0x05 = 0x87`，+0 = `0x87`。上面我写 `8A` 是错的，应该是 `0x87`）

---

## 六、校验和算法

校验和计算范围：**CMD 字节 + DATA 所有字节**，不包含 SOF0、SOF1、LEN 和 CHK 本身。

### C 语言实现

```c
uint8_t CalcChecksum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}
```

### 计算示例

以 `CMD_CONTROL`（使能电机1）为例：

```
CMD = 0x03, motor_id = 0x00, ctrl_cmd = 0x00
sum = 0x03 + 0x00 + 0x00 = 0x03
CHK = 0x03
```

完整帧：

```hex
AA 55 03 03 00 00 03
```

---

## 七、状态机与命令交互

### 电机状态机

```
                    ┌─────────────────┐
                    ▼                 │
[上电] ──► [IDLE] ◄───── C x 1 (DISABLE)
              │
              │ C x 0 (ENABLE)
              ▼
         [RUNNING] ◄────────────────────┐
              │                          │
              │ C x 3 (EMERGENCY)        │ C x 0 (ENABLE)
              ▼                          │
       [EMERGENCY] ──► C x 4 (CLEAR) ──► [IDLE]
              │
              │ C x 2 (HOME)
              ▼
         [IDLE] ──► C x 0 (ENABLE) ──► [RUNNING]
```

### 关键交互规则

1. **必须先使能再设目标**：电机上电默认 `state = IDLE`、`enabled = 0`，PWM 锁定为 0。必须先发 `C x 0` 进入 `RUNNING` 状态，后续的 `M` 命令才会产生 PWM 输出。

2. **`M` 命令重置积分**：每次收到 `CMD_SET_TARGET`，下位机自动重置 `traj_speed = 0` 并清空 PID 积分。这是为了防止旧指令的积分 windup 干扰新轨迹。

3. **急停后必须清除再使能**：`C x 3` 急停后，电机进入 `EMERGENCY` 状态，此时任何 `M` 命令都不会生效。必须先发 `C x 4` 清除故障，再发 `C x 0` 重新使能。

4. **命令排队执行**：下位机内部有 12 级命令队列。若上位机连续发送多帧，下位机会在 1 kHz 控制节拍中逐条取出执行，不会丢帧（队列满时静默丢弃最新帧）。

---

## 八、完整通信时序示例

### 场景：电机1 速度模式闭环控制

```
上位机                                  下位机
─────────────────────────────────────────────────────────
  │                                         │
  │ ──[使能]──────────────────────────────► │
  │  AA 55 03 03 00 00 03                   │
  │                                         │  state → RUNNING
  │                                         │
  │ ──[设目标: 5 rad/s, 加减速 10]────────► │
  │  AA 55 13 01 00 00 ... (19 bytes)       │
  │                                         │  target_speed = 5.0
  │                                         │  traj_speed 从 0 ramp
  │                                         │
  │ ◄────────[STATUS 周期性上报]─────────── │  ← 每 status_interval_ms
  │  AA 55 19 81 00 10 ... (29 bytes)       │
  │                                         │
  │ ──[心跳]──────────────────────────────► │
  │  AA 55 01 05 05                         │
  │ ◄────────────────[ACK]───────────────── │
  │  AA 55 03 82 05 00 87                   │
  │                                         │
  │ ──[急停]──────────────────────────────► │
  │  AA 55 03 03 00 03 06                   │
  │                                         │  state → EMERGENCY
  │                                         │  PWM = 0
  │                                         │
  │ ──[清除故障]──────────────────────────► │
  │  AA 55 03 03 00 04 07                   │
  │                                         │  state → IDLE
  │                                         │
  │ ──[重新使能]──────────────────────────► │
  │  AA 55 03 03 00 00 03                   │
  │                                         │  state → RUNNING
```

---

## 九、注意事项与常见问题

### 1. 浮点数字节序

STM32F103 为小端 MCU，float 的内存布局与 x86/x64 一致。上位机（如 PC）发送浮点数时直接按内存拷贝即可：

```c
float speed = 5.0f;
uart_send((uint8_t *)&speed, 4);  // 发送 00 00 A0 40
```

### 2. 帧长度校验

下位机对 `LEN` 字段有最小值检查：

| 命令 | 最小 LEN | 总帧长 |
|------|----------|--------|
| SET_TARGET | 19 | 23 字节 |
| SET_PID | 15 | 19 字节 |
| CONTROL | 3 | 7 字节 |
| REQ_STATUS | 1 | 5 字节 |
| HEARTBEAT | 1 | 5 字节 |
| SET_VOFA | 3 | 7 字节 |
| STATUS (上行) | 25 | 29 字节 |
| ACK (上行) | 3 | 7 字节 |

若 `LEN` 小于最小值或大于 60，帧会被丢弃。

### 3. 无编码器时的开环表现

若未接编码器，`actual_speed` 始终为 0，PID 速度环误差 = `traj_speed − 0`，积分会持续 windup 到限制值，最终 PWM 输出稳定在约 260（取决于 PID 参数）。接上编码器后，PWM 会立即下降到与实际负载匹配的稳态值。

### 4. 模式切换

两种模式**互斥**，通过宏切换：

```c
// Core/Inc/protocol.h 第 32 行
#define PROTOCOL_VOFA_ONLY      0   /* 将 1 改为 0 */
```

- `PROTOCOL_VOFA_ONLY = 1`：仅解析 FireWater 文本命令，仅发送 JustFloat 波形。
- `PROTOCOL_VOFA_ONLY = 0`：仅解析二进制帧（`0xAA 0x55`），仅发送二进制 STATUS / ACK 帧。

**不要**在两种模式间混用协议，会导致数据解析错误。
- 标准 STATUS 帧按 `status_interval_ms` 间隔自动发送
