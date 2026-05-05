# Unit5 构建、烧录与串口输出

## 环境

- Zephyr 3.7.0: `/home/huiming/zephyrproject/zephyr`
- Zephyr SDK 0.17.0: `/opt/zephyr-sdk-0.17.0`
- 工具链: `arm-zephyr-eabi-gcc` 12.2.0
- 目标板: `robomaster_c` (DJI RoboMaster Type-C, STM32F407IG)

## 构建

```bash
cd /home/huiming/Desktop/Unit5

source zephyr-env.sh
cmake -B build -DBOARD=robomaster_c -DBOARD_ROOT=.
cmake --build build
```

或一键：
```bash
./scripts/build.sh
```

## 烧录

### 方案一：ST-LINK V2（推荐）

```bash
sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 1000" \
  -c "program build/zephyr/zephyr.elf verify reset exit"
```

或一键：
```bash
./scripts/flash_stlink.sh
```

### 方案二：CMSIS-DAP（无线 Horco v0.2）

烧录前先解除 HID 驱动占用：
```bash
echo "3-1:1.1" | sudo tee /sys/bus/usb/drivers/usbhid/unbind
```

```bash
sudo openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 500" \
  -c "program build/zephyr/zephyr.elf verify reset exit"
```

或一键：
```bash
./scripts/flash_cmsis.sh
```

> CMSIS-DAP 无线连接不稳定时，adapter speed 降到 500，烧录失败的话重试或物理重插 USB。

## 串口输出

Console 配置在 **USART6** (PG14/PG9, 115200 baud)。用 USB-TTL (CH340) 连接。

### 物理接线

```
USB-TTL ──────────── C板
  RX  ──────────→ PG14 (USART6_TX)  注意交叉！(板子丝印 UART1)
  TX  ──────────→ PG9  (USART6_RX)
  GND ──────────→ GND
```

### 推荐方式

```bash
picocom -b 115200 /dev/ttyUSB0
# 或
./scripts/serial.sh /dev/ttyUSB0
# Ctrl+A Ctrl+X 退出
```

### 保存数据

```bash
cat /dev/ttyUSB0 > feedback.csv
# 用手拖动电机活动后 Ctrl+C 停止
```

### 可视化

```bash
python tools/plot_feedback.py --file feedback.csv
# 或实时
python tools/plot_feedback.py --port /dev/ttyUSB0 --baud 115200
```

## CSV 格式

```
t_ms, M1_rad, M1_vel, M2_rad, M2_vel, M3_rad, M3_vel, M4_rad, M4_vel
```

- M1/M2 = CAN1 左腿, M3/M4 = CAN2 右腿
- 位置单位 rad, 速度 rad/s

## 故障排查

### 串口无输出

1. 检查接线：RX/TX 必须交叉（USB-TTL RX → PG14, USB-TTL TX → PG9）
2. GND 必须接通
3. 确认波特率 115200
4. 检查 `/dev/ttyUSB0` 是否存在：`ls /dev/ttyUSB*`
5. 杀掉占用进程：`sudo fuser /dev/ttyUSB0`

### 电机无反馈 (rx_count 不增长)

1. 确认电机 24V 供电已接通
2. 检查 CAN 总线接线和终端电阻
3. 用 OpenOCD 诊断：
   ```bash
   # 读取电机状态
   sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
     -c "init" -c "halt" -c "mdw 0x200005CC 24" -c "exit"
   ```
4. 用 `tools/parse_memdump.py` 解析内存数据

### ST-LINK 烧录失败

1. 检查 ST-LINK 是否被识别：`lsusb | grep ST-LINK`
2. 确认 SWD 接线正确（SWDIO/SWCLK/GND）
3. 尝试慢速烧录：adapter speed 降到 100

### CMSIS-DAP 烧录失败 (CMD_INFO)

1. 解除 HID 绑定：`echo "3-1:1.1" | sudo tee /sys/bus/usb/drivers/usbhid/unbind`
2. 物理重插 CMSIS-DAP USB 适配器
3. 目标板断电重上电
4. adapter speed 降到 100
