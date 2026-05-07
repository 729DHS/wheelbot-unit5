# Unit5 — DM4310 串口调试项目

## 项目定位

**通过串口实时读取 DM4310 电机角度，Zephyr Shell 交互控制电机。**

- 上电 bringup 四台电机，极小增益保持 MIT 使能 (KP=0.01, KD=0.001)
- USART6 串口 = Shell 交互 + CSV 角度流 (100Hz)
- 电机可被自由拖动，无力矩输出

## 硬件接线

```
Horco CMSIS-DAP v0.2 (SWD+串口二合一) ── C板/STM32F407
    SWDIO   ──────────→  SWDIO
    SWCLK   ──────────→  SWCLK
    UART RX ──────────→  PG14 (USART6_TX)  注意交叉！
    UART TX ──────────→  PG9  (USART6_RX)
    GND     ──────────→  GND

或分离方案:
  ST-LINK V2 (SWD) + USB-TTL CH340 (串口)
```

- **调试器**: Horco CMSIS-DAP v0.2 (推荐, SWD+串口合一, /dev/ttyACM0)
- **串口**: 板子丝印 UART1 (3-pin) = STM32 USART6 (PG14/PG9), 115200 8N1
- **电机**: DM-J4310-2EC × 4 (CAN1: M1+M2 左腿, CAN2: M3+M4 右腿), 24V 供电

## 快速开始

```bash
cd /home/huiming/Desktop/Projects/Unit5

# 1. 构建
./scripts/build.sh

# 2. 烧录
./scripts/flash_cmsis.sh          # CMSIS-DAP (需物理重插一次)

# 3. 终端仪表盘 (推荐)
python3 tools/monitor_angles.py --port /dev/ttyACM0

# 4. 或打开 Shell
picocom -b 115200 /dev/ttyACM0   # Ctrl+A Ctrl+X 退出
```

## Shell 命令

在 `uart:~$` 提示符下输入:

```
motor enable <1-4|all>    使能电机
motor disable <1-4|all>   失能电机
motor zero <1-4|all>      设置零点
motor csv on              开始 CSV 角度流
motor csv off             停止 CSV 角度流
motor csv once            单次输出当前角度
motor status              查看全部电机状态 (角度/温度/KP/KD)
motor kp <1-4> <value>    设置 KP
motor kd <1-4> <value>    设置 KD
```

Shell 和 `monitor_angles.py` 不能同时使用同一串口。

## 串口输出格式

CSV 通过 `motor csv on` 启用后，与 `uart:~$` 交替输出:

```
t_ms,M1_rad,M2_rad,M3_rad,M4_rad
12345,0.1234,-0.2345,0.4567,-0.7890
```

- 频率: 100Hz (每 10ms), 角度: rad
- 上电 bringup 中 ZERO 步骤自动置零

## 项目结构

```
Unit5/
  CLAUDE.md                   # Claude Code 开发引导
  AGENT.md                    # 本文件
  README.md                   # 项目说明
  CMakeLists.txt              # Zephyr 构建 (main.c + dm4310_motor.c + shell_commands.c)
  prj.conf                    # Kconfig (CAN, Shell, Serial)
  zephyr-env.sh               # Zephyr SDK 环境变量
  boards/dji/robomaster_c/    # 板级 DTS + Kconfig
  src/
    main.c                    # 主循环 (bringup → 5ms tick → CSV 流)
    dm4310_motor.c            # DM4310 MIT 协议驱动 + 单电机控制 API
    dm4310_motor.h            # 协议常量、结构体、API 声明
    shell_commands.c          # Shell 命令注册 (motor enable/disable/csv/status/kp/kd)
  docs/
    4310说明书.md             # DM4310 电机 datasheet 翻译
    STM32_C板用户手册.md      # RM C板 硬件参考 (含 UART 丝印映射)
    build-and-flash.md        # 构建烧录详细步骤
    serial_debug.md           # 串口调试参考 (Ascento 项目, Shell 架构参考)
  tools/
    monitor_angles.py         # 终端实时仪表盘 (自动 CSV 开关, ANSI 转义)
	    serial_bridge.py         # 串口 CSV → UDP 桥接 (数字孪生)
    plot_feedback.py          # matplotlib 角度波形图
    parse_memdump.py          # OpenOCD 内存 dump 解析
    read_feedback_ocd.py      # OpenOCD 直接读内存 (备选)
  scripts/
    build.sh / flash_cmsis.sh / flash_stlink.sh / serial.sh / check_devices.sh
```

## 关键配置

| 配置项 | 文件 | 值 |
|--------|------|-----|
| Console + Shell UART | DTS | USART6 (PG14/PG9) @ 115200 |
| 控制频率 | main.c | 200Hz (CTRL_PERIOD_MS=5) |
| CSV 频率 | main.c | 100Hz (PRINT_PERIOD_MS=10) |
| CAN 波特率 | DTS+DTS | 1Mbps, ONE_SHOT |
| 保持增益 | main.c | KP=0.01, KD=0.001 (≈零力矩) |
| Shell 栈 | prj.conf | 3072 bytes |

## Zephyr 环境

- Zephyr 3.7.0: `/home/huiming/zephyrproject/zephyr`
- Zephyr SDK 0.17.0: `/opt/zephyr-sdk-0.17.0`
- 工具链: `arm-zephyr-eabi-gcc` 12.2.0
- 目标板: `robomaster_c` (STM32F407IG)

## 已知问题 & 排查

1. **CMSIS-DAP CMD_INFO 失败**: 物理重插 USB 适配器, adapter speed 降到 100
2. **电机无反馈**: 检查 24V 供电, CAN 接线, 终端电阻
3. **串口无输出**: 检查接线交叉 (RX→PG14, TX→PG9), GND, 波特率 115200
4. **Shell 输入回显双字符**: picocom 去掉 `--echo` 参数
