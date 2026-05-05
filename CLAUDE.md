# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 Zephyr RTOS 的 DM-J4310-2EC 电机纯反馈读取固件，运行在 DJI RoboMaster Type-C 板 (STM32F407IG) 上。通过双路 CAN 总线控制 4 台电机，每 50ms 通过 USART6 串口 (PG14/PG9) 输出 CSV 格式反馈数据。电机以极小增益 (KP=0.01, KD=0.001) 保持使能，可被自由拖动，不做运动控制。

**这是纯反馈测试固件，无 Shell、无 IMU 传感器逻辑、无平衡/运动控制。** 输出仅依赖 `printk()`，不启用 Zephyr Shell。

## 常用命令

```bash
./scripts/build.sh                    # 构建固件
./scripts/flash_stlink.sh             # ST-LINK V2 烧录 (推荐)
./scripts/flash_cmsis.sh              # CMSIS-DAP 烧录 (备选)
./scripts/serial.sh /dev/ttyUSB0      # 打开串口终端 (优先 picocom)
./scripts/check_devices.sh            # 检测 USB/串口设备
```

可视化：
```bash
python tools/plot_feedback.py --file feedback.csv      # 离线可视化
python tools/plot_feedback.py --port /dev/ttyUSB0       # 实时可视化
```

保存串口数据：
```bash
cat /dev/ttyUSB0 > data.csv          # Ctrl+C 停止后可用 plot_feedback.py 查看
```

## 构建环境

- Zephyr 3.7.0: `/home/huiming/zephyrproject/zephyr`
- Zephyr SDK 0.17.0: `/opt/zephyr-sdk-0.17.0`
- 工具链: `arm-zephyr-eabi-gcc` 12.2.0
- 目标板: `robomaster_c` (boards/dji/robomaster_c/)
- 环境变量在 `zephyr-env.sh` 中定义，`build.sh` 会自动 source

## 架构

```
main.c                           # 主入口: bringup → 50ms CSV 输出循环
  ├── dm4310_poll_rx()           #   从 CAN 消息队列取反馈帧，更新电机状态
  ├── dm4310_hold_positions()    #   写入目标位置 (hold_pos_rad, 饱和赋值=1)
  └── dm4310_tick()              #   发送 CAN 控制帧 (每 tick 4 台电机)

dm4310_motor.c                   # MIT 协议驱动实现
  ├── g_dm4310 (全局状态)        #   包含 bringup 状态机、电机状态、hold 参数
  ├── dm4310_init()              #   初始化 CAN1+CAN2 (1Mbps, ONE_SHOT, 全通滤波)
  ├── drain_rx()                 #   从 K_MSGQ 取出所有 CAN 帧解析反馈
  ├── refresh_online_mask()      #   更新在线位掩码 (超时 500ms 标记离线)
  ├── dm4310_pack_control()      #   浮点参数 → 8 字节 MIT 控制帧编码
  ├── dm4310_decode_feedback()   #   8 字节 CAN 帧 → 电机状态解码
  └── dm4310_stop_all()          #   紧急停止 (四台 DISABLE)

dm4310_motor.h                   # 协议常量、结构体、API 声明
```

**CAN 总线分配**:
- CAN1 (PD0/PD1, 1Mbps): 电机 M1+M2 (左腿)
- CAN2 (PB5/PB6, 1Mbps): 电机 M3+M4 (右腿)
- 消息队列: `K_MSGQ_DEFINE(dm4310_can_rx_msgq, 64 frames)`

**Flash 分区** (DTS):
- `partition@0`: 0x00000-0xC0000 (768KB, 固件)
- `partition@c0000`: 0xC0000-0x100000 (256KB, storage/NVS)

## 关键配置

| 参数 | 值 | 位置 |
|------|-----|------|
| Console UART | USART6 (PG14 TX / PG9 RX) @ 115200 | DTS `chosen.zephyr,console` |
| 控制周期 | 5ms (200Hz) | `main.c:CTRL_PERIOD_MS` |
| 输出周期 | 50ms (20Hz) | `main.c:PRINT_PERIOD_MS` |
| CAN 波特率 | 1Mbps | DTS `&can1/&can2.bitrate` |
| CAN 发送模式 | ONE_SHOT (不自动重试) | `dm4310_motor.c:dm4310_init_bus()` |
| Bringup 后默认 KP/KD | 90.0 / 1.8 | `dm4310_init()` 末段 |
| 反馈输出 KP/KD | 0.01 / 0.001 | `main.c` 第 49-52 行 |
| 在线超时 | 500ms | `DM4310_ONLINE_TIMEOUT_MS` |

**prj.conf 关键项**:
- `CONFIG_CAN=y` — CAN 总线驱动
- `CONFIG_UART_ASYNC_API=y` — USART6 异步模式 (DMA)
- `CONFIG_CBPRINTF_FP_SUPPORT=y` — printk 浮点格式化 (%f)
- `CONFIG_PRINTK=y` — 仅 printk 输出，**无 CONFIG_SHELL**

## 重要行为细节

- **KP=0 且 KD=0 会触发 DISABLE**，不是零力矩 MIT 控制帧。需要极小非零增益 (如 KP=0.01/KD=0.001) 才能保持使能且无力矩输出。见 `dm4310_tick()` 中 bringup_done 后的分支逻辑 (`dm4310_motor.c:546-548`)。
- **hold_updates 使用饱和赋值 (=1) 而非自增**，防止 uint32_t 溢出归零时产生一帧全零控制命令导致电机瞬间失能。见 `dm4310_hold_positions()` (`dm4310_motor.c:698-702`)。
- **Bringup 流程** (每台电机 staggered 执行，每 tick 只处理一台，总计 ~250ms):
  Step 0: 写 MIT 模式寄存器 (StdId 0x7FF, reg 0x0A = 1) → Step 1: DISABLE (10 tick) → Step 2: ZERO (10 tick) → Step 3: ENABLE (10 tick)
- **CAN ID 映射**: 控制帧 ID = `DM4310_CAN_TX_ID_BASE (1) + motor_index`，反馈帧通过 `data[0] & 0x0F` 获取 motor_id 区分 (ID 0 无效，1-4 对应 M1-M4)。
- **反馈帧实际布局与 datasheet 不同**: D0[3:0]=ID, D0[7:4]=状态, D1-D2=位置(大端), D3-D4[7:4]=速度, D4[3:0]-D5=力矩, D6=MOS温度, D7=线圈温度。
- **所有 CAN 发送使用 ONE_SHOT 模式**，发送失败不自动重试。
- **本项目无 Zephyr Shell**，调试只能通过观察串口 CSV 输出。`docs/serial_debug.md` 描述的是另一个项目 (Ascento) 的 Shell 机制，与本项目无关。
- **所有 CS 输出通过 `printk()`**，不是 `printf()` 或 Shell。CSV header 在 bringup 完成后打印一次 (`main.c:54`)。

## 串口输出格式

```
# t_ms,M1_rad,M1_vel,M2_rad,M2_vel,M3_rad,M3_vel,M4_rad,M4_vel
12345,0.1234,0.5678,-0.2345,-0.3456,0.4567,0.6789,-0.7890,-0.8901
```

- 角度单位 rad，速度单位 rad/s，时间戳 ms
- 20Hz 频率 (每 50ms 一行)
- M1/M2 = CAN1 (左腿), M3/M4 = CAN2 (右腿)

## 硬件接线

- 串口: USB-TTL CH340, RX→PG14, TX→PG9, GND→GND (交叉)
  注意: 板子丝印 UART1 (3-pin) 实际对应 STM32 USART6
- 烧录: ST-LINK V2 SWD (SWDIO/SWCLK/GND) 或 CMSIS-DAP
- 电机: 24V 供电, 4 台通过 CAN1/CAN2 连接

## 工具脚本

| 脚本 | 用途 |
|------|------|
| `scripts/build.sh` | cmake 配置 + 编译 |
| `scripts/flash_stlink.sh` | OpenOCD + ST-LINK V2 烧录 |
| `scripts/flash_cmsis.sh` | OpenOCD + CMSIS-DAP 烧录 |
| `scripts/serial.sh [port]` | 打开串口终端 (picocom/minicom/screen) |
| `scripts/check_devices.sh` | 检测 USB/串口设备 |
| `tools/plot_feedback.py` | CSV 反馈数据可视化 (离线/实时) |
| `tools/parse_memdump.py` | OpenOCD 内存 dump 解析 |
| `tools/read_feedback_ocd.py` | OpenOCD 直接读内存 (备选调试) |
