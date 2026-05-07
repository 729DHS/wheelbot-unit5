# Unit5 构建、烧录与串口

## 环境

- Zephyr 3.7.0: `/home/huiming/zephyrproject/zephyr`
- Zephyr SDK 0.17.0: `/opt/zephyr-sdk-0.17.0`
- 工具链: `arm-zephyr-eabi-gcc` 12.2.0
- 目标板: `robomaster_c` (DJI RoboMaster Type-C, STM32F407IG)

## 构建

```bash
./scripts/build.sh
# 产物: build/zephyr/zephyr.elf (~66KB Flash, ~14KB RAM)
```

## 烧录

### 方案一：CMSIS-DAP (推荐, Horco v0.2)

```bash
./scripts/flash_cmsis.sh
```

- 烧录前脚本自动解除 HID 驱动占用
- 如报 `CMD_INFO failed`: **物理重插** CMSIS-DAP USB 适配器，等待 3 秒再试
- adapter speed 默认 500，失败可手动降到 100

### 方案二：ST-LINK V2

```bash
./scripts/flash_stlink.sh
```

## 串口

### 物理连接

CMSIS-DAP 的 UART 接 C板丝印 UART1 (3-pin):

```
CMSIS-DAP UART ──────── C板
    RX      ──────────→  PG14 (USART6_TX)  交叉！
    TX      ──────────→  PG9  (USART6_RX)
    GND     ──────────→  GND
```

如用 USB-TTL (CH340) 同上接线，设备为 `/dev/ttyUSB0`。

### 终端仪表盘 (推荐)

```bash
python3 tools/monitor_angles.py --port /dev/ttyACM0
```

- 自动启用 CSV 流，实时显示 4 路角度 (rad + deg + 进度条)
- Ctrl+C 退出，自动关闭 CSV 流

### Shell 交互

```bash
picocom -b 115200 /dev/ttyACM0     # 不要 --echo
# 按回车 → uart:~$
```

Shell 命令速查:

| 命令 | 说明 |
|------|------|
| `motor enable 1` | 使能 M1 |
| `motor disable all` | 失能全部 |
| `motor zero 2` | M2 置零 |
| `motor csv on` | 开始 CSV 角度流 |
| `motor csv off` | 停止 CSV |
| `motor csv once` | 单次输出角度 |
| `motor status` | 状态详情 (角度/温度/KP/KD) |
| `motor kp 1 0.1` | M1 KP=0.1 |
| `motor kd 3 0.05` | M3 KD=0.05 |

退出: `Ctrl+A` 然后 `Ctrl+X`

### 可视化 (matplotlib)

```bash
python3 tools/plot_feedback.py --port /dev/ttyACM0    # 实时波形
python3 tools/plot_feedback.py --file data.csv        # 离线查看
```

## CSV 格式

```
t_ms,M1_rad,M2_rad,M3_rad,M4_rad
12345,0.1234,-0.2345,0.4567,-0.7890
```

- 100Hz, 角度 rad, 上电 bringup 后自动置零
- M1/M2 = CAN1 (左腿), M3/M4 = CAN2 (右腿)

## GDB 调试 (OpenOCD)

### 交互式调试

```bash
./scripts/debug_gdb.sh [cmsis|stlink]
```

GDB 内置快捷命令:

| 命令 | 说明 |
|------|------|
| `motor_enable 1` | 使能 M1 |
| `motor_disable 2` | 失能 M2 |
| `motor_zero 3` | M3 置零 |
| `motor_status` | 全部状态 |
| `motor_csv` | 单次角度 |
| `motor_kp 1 0.5` | 设置 M1 KP |
| `motor_kd 2 0.05` | 设置 M2 KD |
| `motor_stop` | 紧急停止 |
| `motor_online` | 在线掩码 |

也可直接用 GDB 原生命令: `call dm4310_enable_motor(1)`, `print g_dm4310.motor[0].pos_rad` 等。

### 批处理控制 (免交互)

```bash
python3 tools/gdb_motor.py enable 1
python3 tools/gdb_motor.py disable all
python3 tools/gdb_motor.py status
python3 tools/gdb_motor.py kp 2 0.1
```

每行独立连接 GDB → 执行 → 退出。需 OpenOCD 已在后台运行。

### 内存直读 (无串口)

```bash
sudo python3 tools/read_feedback_ocd.py
```

通过 SWD 直接 `halt → mdw → resume` 读取 `g_dm4310.motor[]` 内存。

## 故障排查

### CMSIS-DAP CMD_INFO 失败

1. **物理重插** CMSIS-DAP USB 适配器 (最有效)
2. 确认目标板已上电
3. adapter speed 降到 100

### 串口无输出

1. 检查接线: RX/TX 必须交叉
2. GND 必须接通
3. 波特率 115200
4. `ls /dev/ttyACM* /dev/ttyUSB*` 确认设备存在
5. `sudo fuser /dev/ttyACM0` 杀掉占用进程

### 电机无反馈

1. 24V 供电接通
2. CAN 总线接线 + 终端电阻
3. `motor status` 查看 online 位掩码和 rx_count

### Shell 输入回显双字符

picocom **不要加** `--echo`，Zephyr Shell 自带远程回显。
