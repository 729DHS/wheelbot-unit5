# Unit5: DM4310 串口调试

> 通过 USART6 串口 Shell 交互控制 + 实时读取 DM4310 电机角度。

## 定位

**电机反馈调试固件。** 上电 bringup 四台电机后，以极小增益保持 MIT 使能，通过 Zephyr Shell 交互控制电机使能/失能/置零，CSV 流输出角度。

## 硬件

- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **电机**: DM-J4310-2EC × 4 (CAN1: M1+M2 左腿, CAN2: M3+M4 右腿)
- **调试器**: Horco CMSIS-DAP v0.2 (SWD + 串口二合一)
- **串口**: 板子丝印 UART1 (3-pin) = USART6 (PG14/PG9), 115200

## 快速开始

```bash
./scripts/build.sh && ./scripts/flash_cmsis.sh

# 终端仪表盘
python3 tools/monitor_angles.py --port /dev/ttyACM0

# 或 Shell 交互
picocom -b 115200 /dev/ttyACM0
```

## Shell 命令

```
motor enable 1          # 使能 M1
motor disable all       # 失能全部
motor zero 2            # M2 置零
motor csv on            # 开始角度流
motor csv off           # 停止
motor status            # 查看状态
motor kp 1 0.05         # 设置 M1 KP
```

## 串口输出 (CSV)

```
t_ms, M1_rad, M2_rad, M3_rad, M4_rad
```

100Hz, 角度 rad, 上电自动置零。

## 更多文档

- `AGENT.md` — 完整项目说明
- `CLAUDE.md` — Claude Code 开发引导
- `docs/build-and-flash.md` — 构建烧录
- `docs/STM32_C板用户手册.md` — 硬件参考 (UART 丝印映射)
