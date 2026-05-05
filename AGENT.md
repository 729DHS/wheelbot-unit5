# Unit5 — DM4310 串口调试项目

## 项目定位

**通过串口实时读取 DM4310 电机角度/速度反馈数据，不做运动控制。**

- 上电 bringup 四台电机 → DISABLE 或极小增益保持使能
- 每 50ms 通过 USART6 串口输出 CSV 格式反馈数据
- 电机可被自由拖动，无力矩输出

## 硬件接线

```
USB-TTL (CH340) ──────── C板/STM32F407
    RX      ──────────→  PG14 (USART6_TX)   注意交叉！(板子丝印 UART1)
    TX      ──────────→  PG9  (USART6_RX)
    GND     ──────────→  GND

ST-LINK V2 ──────────── C板
    SWDIO   ──────────→  SWDIO
    SWCLK   ──────────→  SWCLK
    GND     ──────────→  GND
    3V3     ─── 可选 ──→ 3V3 (仅电压检测)
```

- **串口**: USB-TTL (CH340), 115200 8N1, 连接到 /dev/ttyUSB0 (板子丝印 UART1 = STM32 USART6, PG14/PG9)
- **烧录**: ST-LINK V2 (SWD) 或 CMSIS-DAP (无线 Horco v0.2)
- **电机**: DM-J4310-2EC × 4 (CAN1: M1+M2 左腿, CAN2: M3+M4 右腿), 需要 24V 供电

## 快速开始

```bash
cd /home/huiming/Desktop/Unit5

# 1. 构建
./scripts/build.sh

# 2. 烧录 (ST-LINK)
./scripts/flash_stlink.sh

# 3. 打开串口
./scripts/serial.sh /dev/ttyUSB0
```

## 串口输出格式

```
# t_ms,M1_rad,M1_vel,M2_rad,M2_vel,M3_rad,M3_vel,M4_rad,M4_vel
12345,0.1234,0.5678,-0.2345,-0.3456,0.4567,0.6789,-0.7890,-0.8901
```

- 频率: 20Hz (每 50ms 一行)
- 角度: rad, 速度: rad/s
- 可直接 `cat /dev/ttyUSB0 > data.csv` 保存后用 plot_feedback.py 可视化

## 项目结构

```
Unit5/
  AGENT.md                    # 本文件 — Agent 引导
  README.md                   # 项目说明
  CMakeLists.txt              # Zephyr 构建配置
  prj.conf                    # Zephyr Kconfig
  zephyr-env.sh               # Zephyr 环境变量
  boards/dji/robomaster_c/    # 板级定义 (DTS, Kconfig)
  src/
    main.c                    # 主入口 — 反馈测试固件
    dm4310_motor.c            # DM4310 MIT 协议驱动
    dm4310_motor.h            # 驱动头文件
  docs/
    build-and-flash.md        # 构建与烧录详细说明
    serial_debug.md           # 串口调试经验参考
  tools/
    parse_memdump.py          # OpenOCD 内存转储解析
    plot_feedback.py          # 反馈数据可视化
    read_feedback_ocd.py      # OpenOCD 直接读内存 (备选)
  scripts/
    build.sh                  # 构建
    flash_cmsis.sh            # CMSIS-DAP 烧录
    flash_stlink.sh           # ST-LINK 烧录
    serial.sh                 # 串口终端
    check_devices.sh          # 设备检测
```

## 关键配置

| 配置项 | 文件 | 值 |
|--------|------|-----|
| Console UART | DTS | USART6 (PG14/PG9) @ 115200 |
| 电机控制模式 | main.c | MIT, KP=0.01/KD=0.001 (≈零力矩) |
| 反馈输出频率 | main.c | 20Hz (PRINT_PERIOD_MS=50) |
| 控制频率 | main.c | 200Hz (CTRL_PERIOD_MS=5) |
| CAN 波特率 | dm4310_motor.c | 1 Mbps, ONE_SHOT 模式 |

## Zephyr 环境

- Zephyr 3.7.0: `/home/huiming/zephyrproject/zephyr`
- Zephyr SDK 0.17.0: `/opt/zephyr-sdk-0.17.0`
- 工具链: `arm-zephyr-eabi-gcc` 12.2.0
- 目标板: `robomaster_c` (DJI RoboMaster Type-C, STM32F407IG)

## 驱动关键行为

- `dm4310_init()`: 初始化 CAN1/CAN2 (1Mbps, ONE_SHOT), 默认 KP=90/KD=1.8
- `dm4310_tick()`: 
  - bringup_done=0 → 执行 bringup 状态机 (MIT寄存器 → DISABLE → ZERO → ENABLE)
  - bringup_done=1 → 根据 KP/KD 决定发送 MIT 控制帧或 DISABLE
  - KP==0 && KD==0 → 发送 DISABLE 特殊帧 (电机退出 MIT 模式)
  - KP!=0 || KD!=0 && hold_updates>0 → 发送 MIT 控制帧
- `dm4310_poll_rx()`: 从 CAN 消息队列取帧，解析反馈数据

## 已知问题 & 排查

1. **串口无输出**: 检查 USB-TTL 接线 (交叉 RX/TX), GND 必须接, 波特率 115200
2. **电机无反馈 (rx_count=1)**: 检查电机 24V 供电, CAN 总线接线, 终端电阻
3. **CMSIS-DAP CDC 桥不稳定**: 改用 USB-TTL 串口线 (本项目推荐方案)
4. **CMSIS-DAP 烧录失败 (CMD_INFO)**: 物理重插 USB, 解除 HID 绑定
5. **电机蹬腿**: KP/KD 过大, 当前固件使用 KP=0.01/KD=0.001 (安全)
