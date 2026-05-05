# Unit5: DM4310 串口调试

> 通过 USART6 串口 (板子丝印 UART1, PG14/PG9) 实时读取 DM4310 电机角度/速度反馈数据。

## 定位

**纯反馈读取，不做运动控制。** 上电 bringup 四台 DM4310 电机后，以极小增益保持 MIT 使能状态，每 50ms 通过串口输出角度和速度。

与 Unit4 的区别：
| | Unit4 | Unit5 |
|------|-------|-------|
| 目标 | 左右腿同步控制 | 纯串口反馈读取 |
| 电机控制 | 左腿主动跟随 | 全部极小增益（≈零力矩）|
| 串口 | CMSIS-DAP CDC 桥 (不稳定) | USB-TTL (CH340) 直连 |
| 烧录 | CMSIS-DAP | ST-LINK V2 / CMSIS-DAP |
| 复杂度 | IK/FK/同步控制 | 最简反馈测试 |

## 硬件

- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **电机**: DM-J4310-2EC × 4 (CAN1: 左腿, CAN2: 右腿)
- **串口**: USB-TTL (CH340) → PG14/PG9 (USART6), 115200 8N1 (板子丝印 UART1)
- **烧录**: ST-LINK V2 (SWD) 或 CMSIS-DAP

## 快速开始

```bash
cd /home/huiming/Desktop/Unit5
./scripts/build.sh                    # 构建
./scripts/flash_stlink.sh             # 烧录 (ST-LINK)
./scripts/serial.sh /dev/ttyUSB0      # 打开串口
```

## 串口输出

CSV 格式，20Hz：
```
t_ms, M1_rad, M1_vel, M2_rad, M2_vel, M3_rad, M3_vel, M4_rad, M4_vel
```

## 更多文档

- `AGENT.md` — Agent 引导和完整项目说明
- `docs/build-and-flash.md` — 构建烧录详细步骤
- `docs/serial_debug.md` — 串口调试经验参考
