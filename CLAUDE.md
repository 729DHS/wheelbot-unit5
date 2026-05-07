# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 Zephyr RTOS 的 DM-J4310-2EC 电机反馈固件，运行于 DJI RoboMaster Type-C 板 (STM32F407IG)。双路 CAN 控制 4 台电机，USART6 串口提供 Zephyr Shell 交互调试 + CSV 角度流输出。电机以极小增益 (KP=0.01, KD=0.001) 保持使能，可被自由拖动，不做运动控制。

**串口 Shell + CSV 流共存于 USART6**，通过 Shell 命令控制电机和 CSV 开关。

## 常用命令

```bash
./scripts/build.sh                              # 构建
./scripts/flash_stlink.sh                       # ST-LINK V2 烧录 (推荐)
./scripts/flash_cmsis.sh                        # CMSIS-DAP 烧录 (Horco v0.2)
./scripts/serial.sh /dev/ttyACM0                # 打开串口 Shell (picocom)
./scripts/check_devices.sh                      # 检测 USB/串口设备
```

终端仪表盘:
```bash
python3 tools/monitor_angles.py --port /dev/ttyACM0    # 实时角度仪表盘 (推荐)
```

可视化:
```bash
python3 tools/plot_feedback.py --file feedback.csv     # 离线绘图
python3 tools/plot_feedback.py --port /dev/ttyACM0     # 实时绘图
```

## 构建环境

- Zephyr 3.7.0: `/home/huiming/zephyrproject/zephyr`
- Zephyr SDK 0.17.0: `/opt/zephyr-sdk-0.17.0`
- 工具链: `arm-zephyr-eabi-gcc` 12.2.0
- 目标板: `robomaster_c` (boards/dji/robomaster_c/)
- 环境变量: `zephyr-env.sh`, `build.sh` 自动 source

## 架构

```
main.c                           # 主入口: bringup → 主循环 (5ms)
  ├── dm4310_poll_rx()           #   CAN 消息队列 → 电机状态
  ├── dm4310_hold_positions()    #   位置保持 (饱和赋值=1)
  ├── dm4310_tick()              #   发送 CAN 控制帧 (每 tick 4 台)
  └── shell_print(CSV)           #   g_csv_enabled 时输出角度流

dm4310_motor.c                   # MIT 协议驱动
  ├── g_dm4310 (全局状态)        #   bringup 状态机、电机状态、hold 参数
  ├── dm4310_init()              #   CAN1+CAN2 (1Mbps, ONE_SHOT, 全通滤波)
  ├── drain_rx()                 #   K_MSGQ 取 CAN 帧解析反馈
  ├── refresh_online_mask()      #   在线位掩码 (超时 500ms)
  ├── dm4310_pack_control()      #   浮点 → 8 字节 MIT 控制帧
  ├── dm4310_decode_feedback()   #   8 字节 CAN 帧 → 电机状态
  ├── dm4310_enable_motor()      #   单电机使能 (Shell 用)
  ├── dm4310_disable_motor()     #   单电机失能 (Shell 用)
  ├── dm4310_zero_motor()        #   单电机置零 (Shell 用)
  └── dm4310_stop_all()          #   紧急停止 (四台 DISABLE)

shell_commands.c                 # Zephyr Shell 命令注册
  ├── motor enable <1-4|all>     #   使能电机
  ├── motor disable <1-4|all>    #   失能电机
  ├── motor zero <1-4|all>       #   置零点
  ├── motor csv <on|off|once>    #   CSV 角度流开关
  ├── motor status               #   全部电机状态
  ├── motor kp <1-4> <value>     #   设置 KP
  └── motor kd <1-4> <value>     #   设置 KD

dm4310_motor.h                   # 协议常量、结构体、API 声明
```

**CAN 总线分配**:
- CAN1 (PD0/PD1, 1Mbps): M1+M2 (左腿)
- CAN2 (PB5/PB6, 1Mbps): M3+M4 (右腿)
- 消息队列: `K_MSGQ_DEFINE(dm4310_can_rx_msgq, 64 frames)`

## 关键配置

| 参数 | 值 | 位置 |
|------|-----|------|
| Console + Shell UART | USART6 (PG14 TX / PG9 RX) @ 115200 | DTS `chosen` |
| 控制周期 | 5ms (200Hz) | `main.c:CTRL_PERIOD_MS` |
| CSV 输出周期 | 10ms (100Hz) | `main.c:PRINT_PERIOD_MS` |
| CAN 波特率 | 1Mbps | DTS `&can1/&can2.bitrate` |
| CAN 发送模式 | ONE_SHOT | `dm4310_motor.c:dm4310_init_bus()` |
| 默认 KP/KD (bringup 后) | 0.01 / 0.001 | `main.c` |
| 在线超时 | 500ms | `DM4310_ONLINE_TIMEOUT_MS` |

**prj.conf 关键项**: `CONFIG_CAN=y`, `CONFIG_UART_ASYNC_API=y`, `CONFIG_CBPRINTF_FP_SUPPORT=y`, `CONFIG_SHELL=y`, `CONFIG_KERNEL_SHELL=y`

## Shell 与 CSV 共存机制

- Shell 和 CSV 角度流共享 USART6，**默认 CSV 关闭**
- `motor csv on` → 主循环通过 `shell_print()` 输出角度流，与 `uart:~$` 提示符交替
- `motor csv off` → 停止流，Shell 干净可用
- `monitor_angles.py` 自动发 `motor csv on`，退出时发 `motor csv off`
- Shell 和监控工具**不能同时开**（共用同一串口）

## 重要行为细节

- **KP=0 且 KD=0 会触发 DISABLE**。需极小非零增益 (KP=0.01/KD=0.001) 保持使能且无力矩。见 `dm4310_tick()` (`dm4310_motor.c:546-548`)
- **hold_updates 饱和赋值 (=1) 而非自增**，防溢出归零导致电机瞬间失能。见 `dm4310_hold_positions()` (`dm4310_motor.c:698-702`)
- **Bringup 流程** (staggered 执行，每 tick 一台，~250ms):
  Step 0: 写 MIT 模式寄存器 (StdId 0x7FF, reg 0x0A=1) → Step 1: DISABLE (10 tick) → Step 2: ZERO (10 tick) → Step 3: ENABLE (10 tick)
- **CAN ID 映射**: 控制帧 ID = `DM4310_CAN_TX_ID_BASE (1) + motor_index`，反馈帧 `data[0] & 0x0F` 取 motor_id (0 无效, 1-4=M1-M4)
- **反馈帧布局与 datasheet 不同**: D0[3:0]=ID, D0[7:4]=状态, D1-D2=位置(大端), D3-D4[7:4]=速度, D4[3:0]-D5=力矩, D6=MOS温度, D7=线圈温度
- **Shell 命令 `motor enable`** 发送 ENABLE + 设置 hold_kp=0.01/kd=0.001；`motor disable` 发送 DISABLE + 清零增益

## 串口输出格式

```
# CSV header (motor csv on 后通过 shell_print 输出)
t_ms,M1_rad,M2_rad,M3_rad,M4_rad
12345,0.1234,-0.2345,0.4567,-0.7890
```

- 角度 rad，上电 bringup (ZERO) 后当前位置自动置零
- 100Hz (每 10ms)，Shell 和 CSV 交替输出
- M1/M2 = CAN1 (左腿), M3/M4 = CAN2 (右腿)

## 调试方法 (三层递进)

### 1. 串口 Shell (最轻量)

```bash
picocom -b 115200 /dev/ttyACM0
# uart:~$ motor enable 1
# uart:~$ motor status
# uart:~$ motor csv on
```

Shell 和 CSV 流共享 USART6，无需暂停目标。

### 2. GDB + OpenOCD (最强大)

```bash
# 启动交互 GDB 会话
./scripts/debug_gdb.sh [cmsis|stlink]

# GDB 内快捷命令:
#   motor_enable 1       motor_disable 2
#   motor_zero 3         motor_status
#   motor_csv            motor_stop
#   motor_kp 1 0.5       motor_kd 2 0.05
```

GDB 可 `call` 任意函数、`set` 任意变量、设断点，无需修改固件。

### 3. GDB 批处理 (一行命令)

```bash
# 需先启动 OpenOCD (./scripts/debug_gdb.sh 或独立启动)
python3 tools/gdb_motor.py status          # 查看全部状态
python3 tools/gdb_motor.py enable 1        # 使能 M1
python3 tools/gdb_motor.py disable all     # 失能全部
python3 tools/gdb_motor.py zero 2          # M2 置零
python3 tools/gdb_motor.py kp 1 0.5        # M1 KP=0.5
python3 tools/gdb_motor.py stop            # 紧急停止
```

每行命令自动连接 GDB → 执行 → 退出，适合脚本化。

### 4. OpenOCD 内存直读 (无串口时)

```bash
sudo python3 tools/read_feedback_ocd.py    # 通过 SWD 读电机状态
```

原理: OpenOCD `halt → mdw 读内存 → resume`，不经过串口/USART。

## 硬件接线

- 调试器: Horco CMSIS-DAP v0.2 (SWD + 串口二合一, /dev/ttyACM0)
  或 ST-LINK V2 (SWD) + USB-TTL (CH340, /dev/ttyUSB0)
- 串口: 板子丝印 UART1 (3-pin) = STM32 USART6 (PG14 TX / PG9 RX), 115200
- 烧录: SWD (SWDIO/SWCLK/GND)
- 电机: 24V 供电, 4 台通过 CAN1/CAN2 连接

## 工具脚本

| 脚本 | 用途 |
|------|------|
| `scripts/build.sh` | cmake 配置 + 编译 |
| `scripts/flash_stlink.sh` | OpenOCD + ST-LINK V2 烧录 |
| `scripts/flash_cmsis.sh` | OpenOCD + CMSIS-DAP 烧录 (自动解除 HID) |
| `scripts/serial.sh [port]` | 串口终端 (picocom) |
| `scripts/check_devices.sh` | USB/串口设备检测 |
| `tools/monitor_angles.py` | 终端实时仪表盘 (自动 CSV 开关) |
| `tools/plot_feedback.py` | matplotlib 角度波形 (离线/实时) |
| `tools/serial_bridge.py` | 串口 CSV → UDP 桥接 (数字孪生用) |
| `scripts/debug_gdb.sh` | GDB + OpenOCD 交互调试 |
| `tools/gdb_motor.py` | GDB 批处理电机控制 (免交互) |
| `tools/read_feedback_ocd.py` | OpenOCD 内存直读电机状态 |
| `tools/parse_memdump.py` | 解析 OpenOCD mdw 内存转储 |
