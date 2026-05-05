下面是串口调试和烧录的完整说明：

---

## 串口调试

这个项目的调试核心思想是：**不需要改代码重新编译，所有调试命令都在串口 Shell 里输入**。

### 物理接线

```
USB-TTL (CH340) ──────── C板/STM32
    RX      ──────────→  PA9  (USART1_TX)    注意交叉！
    TX      ──────────→  PB7  (USART1_RX)
    GND     ──────────→  GND
波特率：115200, 8N1
```

### 打开串口终端

```bash
./scripts/serial.sh          # 自动找第一个串口
./scripts/serial.sh /dev/ttyUSB0  # 手动指定
```

脚本内部优先用 `picocom --echo`（带本地回显），备选 `minicom`、`screen`。按回车会看到：

```
uart:~$
```

这就是 Zephyr Shell 的提示符，所有调试命令在这里输入，不是 Linux 终端。

### 怎么不用改代码就能调电机

这套机制的关键是 **debug 模式抢占控制线程**。正常时，1kHz 控制线程跑平衡算法（读 IMU → 算 PID → 发 CAN 命令给电机）。当你通过 Shell 发一条调试命令（比如 `motor dm pos left 0.1 1.0 800`），流程是：

1. Shell 命令处理函数调用 `enter_motor_debug_mode()` → 关闭平衡控制
2. 把目标值（位置 0.1 rad、速度 1.0 rad/s、持续 800ms）写入全局 debug state
3. 控制线程每周期先检查：debug 是否活跃？是 → 发 debug 指令而不是平衡指令
4. 800ms 超时后，自动发零力矩指令，退出 debug 模式

所有调试命令都有超时保护（默认 1000ms，最长 5000ms），不会让电机一直转。

### Shell 命令树一览

```
robot enable <0|1>          # 启停平衡
robot height <32..80>       # 设定高度
robot status                # 查看全部状态（pitch/roll/电流/高度/电池）
robot stop                  # 急停

motor can status all        # 看 CAN1/CAN2 有没有 bus-off
motor can recover all       # 手动恢复 CAN 总线

motor dm status all         # 左右关节：位置/速度/扭矩/温度
motor dm enable left        # 使能左关节
motor dm pos left 0.1 1.0 800   # 位置模式转 0.1 rad
motor dm mit left 0 0 20 0.5 0 800  # MIT 模式：大 kp 刚性保持
motor dm reg left 0x0a      # 读寄存器：控制模式
motor dm diag left          # 一键诊断所有关键寄存器
motor dm stop left          # 停止

motor wheel status all      # 左右轮：ERPM/角度/速度/电流
motor wheel current left 300 500  # 300mA 电流转 500ms
motor wheel pair 300 300 500     # 左右轮同时转
motor wheel stop            # 停止

motor debug stop            # 全部调试输出清零
```

### Zephyr 是怎么把串口变成 Shell 的

三层配置联动：

| 层 | 配置 | 作用 |
|---|---|---|
| **设备树** | `chosen { zephyr,shell-uart = &usart1; }` | 指定用 USART1 |
| **Kconfig** | `CONFIG_SHELL=y` + `CONFIG_UART_CONSOLE=y` | 启用 Shell 框架 |
| **代码** | `SHELL_CMD_REGISTER(motor, &motor_cmds, ...)` | 注册命令树 |

命令实现用 `shell_print(sh, ...)` 输出，保证文字送到正确的 shell 实例。

---

## 烧录/下载

### 物理接线（SWD）

```
ST-LINK V2 ────────────   C板
    SWDIO   ──────────→  SWDIO
    SWCLK   ──────────→  SWCLK
    GND     ──────────→  GND
    3V3     ──────────→  3V3 (仅电压检测，不给车供电)
    NRST    ─── 可选 ──→ NRST (推荐接上)
```

### 烧录命令

```bash
./scripts/build.sh           # 先编译
./scripts/flash.sh           # 烧录（推荐，halt→write→verify→reset, ~13s）
```

`flash.sh` 内部做的事：
1. 用 `lsusb` 检查 ST-LINK 是否识被到
2. 调 `west flash --runner openocd -- --cmd-reset-halt halt --verify`
3. OpenOCD 连 ST-LINK → halt CPU → 写 Flash → 校验 → reset → 退出

### 多套烧录方案（按问题从简单到极端）

| 脚本 | 适用场景 |
|---|---|
| `flash.sh` | 日常首选，halt 后烧录 |
| `openocd_flash.sh` | west 有问题时，直接调 OpenOCD |
| `openocd_flash_fast_4pin.sh` | 只有 4 线 SWD，没接 NRST |
| `openocd_flash_bin_slow.sh` | "flash write algorithm aborted" 时，50kHz SWD 慢写 |
| `openocd_mass_erase_flash.sh` | 极致情况，全片擦除再写（6s） |
| `stlink_flash.sh` | 不用 OpenOCD，直接用 stlink-tools |

### 完整开发循环

```bash
vim src/xxx.c                # 改代码
./scripts/build.sh           # 编译
./scripts/flash.sh           # 烧录
./scripts/serial.sh          # 打开串口 Shell
# 在 uart:~$ 里敲命令测试



# 串口调试实现架构

> 版本：2026-05-05
> 概述：Ascento 轮腿机器人的串口调试系统完整实现说明，涵盖硬件连接、框架配置、命令注册、控制接口、调试模块和日志系统。

---

## 目录

1. [总体架构](#1-总体架构)
2. [硬件层：物理串口连接](#2-硬件层物理串口连接)
3. [Zephyr 框架层：Kconfig 与 DTS 配置](#3-zephyr-框架层kconfig-与-dts-配置)
4. [命令层：Shell 命令注册机制](#4-命令层shell-命令注册机制)
5. [控制层：control_* API](#5-控制层control_-api)
6. [调试层：motor_debug_* API](#6-调试层motor_debug_-api)
7. [日志层：周期性状态输出](#7-日志层周期性状态输出)
8. [完整调用链](#8-完整调用链)
9. [串口命令速查](#9-串口命令速查)
10. [使用注意事项](#10-使用注意事项)
11. [关联文档](#11-关联文档)

---

## 1. 总体架构

串口调试系统分为五层，自底向上：

```
┌─────────────────────────────────────────────────────────┐
│  用户输入命令 / 自动日志输出                               │  ← 串口终端 (picocom)
├─────────────────────────────────────────────────────────┤
│  Zephyr Shell 线程 + Logging 后端                        │  ← CONFIG_SHELL / CONFIG_LOG
├─────────────────────────────────────────────────────────┤
│  shell_commands.c                                       │  ← 命令解析与分发
│    ├─ robot → control_*() API (pid_balance_control.c)   │
│    └─ motor → motor_debug_*() API (motor_debug.c)       │
├─────────────────────────────────────────────────────────┤
│  控制线程 (main.c)                                       │  ← 实时控制循环
│    ├─ 正常模式：balance controller → CAN 发送             │
│    └─ 调试模式：motor_debug_get_output() → CAN 直接发送   │
├─────────────────────────────────────────────────────────┤
│  硬件：USART1 (PA9 TX / PB7 RX) @ 115200 baud          │  ← DJI 板载 USB-UART 桥
└─────────────────────────────────────────────────────────┘
```

---

## 2. 硬件层：物理串口连接

### 2.1 UART 配置

**文件**：`boards/arm/dji_f407igh6_c/dji_f407igh6_c.dts`

DJI RoboMaster Development Board Type C（STM32F407IGH6）使用 USART1 作为控制台串口：

```dts
chosen {
    zephyr,console = &usart1;
    zephyr,shell-uart = &usart1;
};
```

引脚定义：

| 功能 | 引脚 | 复用 |
|------|------|------|
| TX | PA9 | AF7 |
| RX | PB7 | AF7, 内部上拉 |

波特率：**115200**，格式：8N1

```dts
&usart1 {
    pinctrl-0 = <&usart1_tx_pa9 &usart1_rx_pb7>;
    pinctrl-names = "default";
    current-speed = <115200>;
    status = "okay";
};
```

### 2.2 物理连接方式

DJI 板载 USB 转串口芯片，USB 插入后在主机上枚举为虚拟串口设备（`/dev/ttyUSB0` 或 `/dev/serial/by-id/...`）。

### 2.3 连接命令

```bash
# 自动查找设备并连接
./scripts/serial.sh

# 手动指定设备
./scripts/serial.sh /dev/ttyUSB0 115200
```

退出：`Ctrl+A` 然后 `Ctrl+X`（picocom）

---

## 3. Zephyr 框架层：Kconfig 与 DTS 配置

**文件**：`prj.conf`

### 3.1 串口与控制台

```ini
CONFIG_SERIAL=y            # UART 驱动
CONFIG_CONSOLE=y           # 控制台子系统
CONFIG_UART_CONSOLE=y      # 控制台路由到 UART（非 RTT/SWO）
CONFIG_PRINTK=y            # printk() 支持
```

### 3.2 Shell 子系统

```ini
CONFIG_SHELL=y             # Zephyr Shell 子系统
CONFIG_KERNEL_SHELL=y      # 内置 kernel 命令（kernel threads, uptime 等）
CONFIG_SHELL_STACK_SIZE=4096
```

`zephyr,shell-uart = &usart1`（DTS chosen 节点）将 Shell 后端绑定到 USART1。Zephyr 自动创建 Shell 线程，从该 UART 读取输入、写入输出。

### 3.3 日志子系统

```ini
CONFIG_LOG=y               # Zephyr 日志子系统
CONFIG_LOG_DEFAULT_LEVEL=3 # INFO 级别
CONFIG_LOG_MODE_IMMEDIATE=y # 无缓冲，同步输出
```

`CONFIG_LOG_MODE_IMMEDIATE=y` 使日志在调用者上下文中同步写入 UART，而非异步日志线程。这导致 `[app]` 和 `[ascento]` 日志会与 Shell 输出交错。

### 3.4 浮点支持

```ini
CONFIG_NEWLIB_LIBC=y
CONFIG_NEWLIB_LIBC_FLOAT_PRINTF=y  # printf %f 支持
CONFIG_CBPRINTF_FP_SUPPORT=y       # cbprintf 浮点支持
```

Shell 命令中大量使用 `%f` 格式输出浮点数，必须启用这些选项。

---

## 4. 命令层：Shell 命令注册机制

**文件**：`src/shell_commands.c`

### 4.1 注册方式

使用 Zephyr 的 `SHELL_CMD_REGISTER` 宏注册两个顶层命令树：

```c
// 顶层命令：robot
SHELL_CMD_REGISTER(robot, &robot_cmds, "wheel-leg robot control", NULL);

// 顶层命令：motor
SHELL_CMD_REGISTER(motor, &motor_cmds, "single motor debug", NULL);
```

`SHELL_STATIC_SUBCMD_SET_CREATE` 构建静态子命令数组，`SHELL_CMD` 嵌套子命令集。`SHELL_CMD_REGISTER` 创建文件作用域构造函数，Zephyr Shell 线程在启动时自动发现所有已注册命令，无需显式调用 `shell_init()`。

### 4.2 robot 命令树

```c
SHELL_STATIC_SUBCMD_SET_CREATE(robot_cmds,
    SHELL_CMD_ARG(enable,  NULL, "robot enable <0|1|on|off>",      cmd_robot_enable,  2, 0),
    SHELL_CMD_ARG(height,  NULL, "robot height <32..80>",           cmd_robot_height,  2, 0),
    SHELL_CMD_ARG(joy,     NULL, "robot joy <x:-100..100> <y:-100..100>", cmd_robot_joy, 3, 0),
    SHELL_CMD_ARG(motion,  NULL, "robot motion <forward|back|left|right|jump|stop>", cmd_robot_motion, 2, 0),
    SHELL_CMD_ARG(stop,    NULL, "robot stop",                      cmd_robot_stop,    1, 0),
    SHELL_CMD_ARG(jump,    NULL, "robot jump",                      cmd_robot_jump,    1, 0),
    SHELL_CMD_ARG(zero,    NULL, "robot zero <deg|now>",            cmd_robot_zero,    2, 0),
    SHELL_CMD_ARG(status,  NULL, "robot status",                    cmd_robot_status,  1, 0),
    SHELL_CMD_ARG(pid,     NULL, "robot pid [angle_p gyro_p ...]",  cmd_robot_pid,     1, 5),
    SHELL_CMD_ARG(param,   NULL, "robot param [list|save|reset|<name> <value>]", cmd_robot_param, 1, 2),
    SHELL_SUBCMD_SET_END);
```

### 4.3 motor 命令树

```c
SHELL_STATIC_SUBCMD_SET_CREATE(motor_cmds,
    SHELL_CMD(wheel, &motor_wheel_cmds, "VESC/M3508 wheel motor debug", NULL),
    SHELL_CMD(dm,    &motor_dm_cmds,    "DM4340 joint motor debug",     NULL),
    SHELL_CMD(debug, &motor_debug_cmds, "manual motor debug state",     NULL),
    SHELL_CMD(can,   &motor_can_cmds,   "CAN bus debug",                NULL),
    SHELL_SUBCMD_SET_END);
```

子命令组：

| 命令组 | 子命令 |
|--------|--------|
| `motor wheel` | `status`, `current`, `rpm`, `pair`, `stop` |
| `motor dm` | `status`, `enable`, `disable`, `zero`, `reg`, `diag`, `pos`, `vel`, `mit`, `nudge`, `wiggle`, `stop`, `rxlog` |
| `motor debug` | `status`, `stop` |
| `motor can` | `status`, `raw`, `rawx`, `recover` |

### 4.4 调试模式入口

所有 `motor` 调试命令在执行前自动调用：

```c
static void enter_motor_debug_mode(void)
{
    control_set_enable(false);   // 关闭平衡控制器
    control_stop_motion();       // 停止所有运动指令
}
```

确保平衡控制器不会与直接电机命令冲突。

---

## 5. 控制层：control_* API

**头文件**：`src/control.h`
**实现文件**：`src/pid_balance_control.c`（当前活跃；`src/control.c` 未编译）

### 5.1 架构

`control_*` API 是平衡控制器的线程安全门面。所有函数使用 `k_mutex` 保护静态 `pid_balance_ctx_t ctx` 结构体。

### 5.2 Shell 调用的关键函数

| 函数 | 作用 | Shell 命令 |
|------|------|-----------|
| `control_set_enable(bool)` | 开启/关闭平衡控制器。disable 时重置所有 PID，enable 时清除 fault | `robot enable` |
| `control_set_height(int)` | 设置目标腿高（钳位到 MIN..MAX） | `robot height` |
| `control_set_joystick(float x, float y)` | 设置摇杆输入（钳位 -100..100） | `robot joy` |
| `control_set_motion(robot_motion_t)` | 设置运动方向 | `robot motion` |
| `control_stop_motion()` | 归零摇杆 + 设为 STOP | `robot stop` |
| `control_set_angle_zero(float)` | 设置平衡零点偏移，重置 PID | `robot zero` |
| `control_get_status(control_status_t*)` | 返回状态快照（线程安全拷贝） | `robot status` |
| `control_get_pid_balance_params(...)` | 返回当前 PID 增益 | `robot pid`（读） |
| `control_set_pid_balance_params(...)` | 应用新 PID 增益（带钳位），重置 PID | `robot pid`（写） |

### 5.3 实时控制循环

`control_step()` 是实时平衡循环函数，由 `main.c` 中的 `control_thread` 以 `APP_CONTROL_HZ` 频率调用。它读取 IMU + 轮电机反馈，输出轮电流 + 关节位置。

---

## 6. 调试层：motor_debug_* API

**头文件**：`src/motor_debug.h`
**实现文件**：`src/motor_debug.c`

### 6.1 架构

`motor_debug` 模块提供**限时直接电机命令**，用于台架测试和标定，绕过平衡控制器。

每个 set 命令在 `motor_debug_state_t` 结构体中存储命令 + 截止时间，受 `k_spinlock` 保护。`control_thread` 每个 tick 调用 `motor_debug_get_output()`；如果有活跃的调试命令，则执行调试命令而非平衡控制器输出。命令在到期后自动清除。

### 6.2 轮电机函数（VESC/M3508, CAN2）

| 函数 | 作用 |
|------|------|
| `motor_debug_set_wheel_current(id, current, duration)` | 设置单轮固定电流 (mA) |
| `motor_debug_set_wheel_pair(left, right, duration)` | 左右轮同时送电流 |
| `motor_debug_set_wheel_rpm(id, rpm, duration)` | 设置轮目标 eRPM |
| `motor_debug_stop_wheels()` | 立即归零所有轮电流 |
| `motor_debug_get_m3508(id, out)` | 读取 M3508 电机反馈 |

### 6.3 关节电机函数（DM4340, CAN1）

| 函数 | 作用 |
|------|------|
| `motor_debug_dm_enable(id)` / `dm_disable(id)` | 使能/失能 DM4340 |
| `motor_debug_dm_save_zero(id)` | 保存当前位置为机械零点 |
| `motor_debug_dm_read_reg(id, rid, out)` | 读取 DM4340 寄存器（阻塞，80ms 超时） |
| `motor_debug_set_dm_pos_vel(id, pos, vel, duration)` | 位置-速度控制 |
| `motor_debug_set_dm_velocity(id, vel, duration)` | 纯速度控制 |
| `motor_debug_set_dm_mit(id, pos, vel, kp, kd, torque, duration)` | MIT 模式（全阻抗控制） |
| `motor_debug_set_dm_mit_raw(...)` | MIT 模式（无关节位置安全钳位） |
| `motor_debug_set_dm_wiggle(id, center, amp, period, kp, kd, duration)` | 正弦摆动 |
| `motor_debug_stop_dm(id)` | 停止单个 DM4340 |
| `motor_debug_get_dm4340(id, out)` | 读取 DM4340 电机反馈 |

### 6.4 全局函数

| 函数 | 作用 |
|------|------|
| `motor_debug_stop_all()` | 停止所有调试，归零所有状态和命令 |
| `motor_debug_get_output(out)` | 控制线程调用，返回当前调试命令（过期自动清除） |

### 6.5 与控制循环的集成

`main.c` 中的控制线程（约第 407-421 行）：

```c
motor_debug_output_t debug;
if (motor_debug_get_output(&debug)) {
    debug_was_active = true;
    send_debug_output(&debug);    // CAN 直接发送
    send_control_joints(&out);    // 关节仍跟随平衡控制器
    ...
    continue;                     // 跳过正常平衡输出
}
```

调试模式激活时，平衡控制器的轮输出被完全绕过。

### 6.6 超时机制

| 常量 | 说明 |
|------|------|
| `APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS` | 默认命令持续时间 |
| `APP_MOTOR_DEBUG_MAX_TIMEOUT_MS` | 最大命令持续时间 |

命令到期后 `motor_debug_get_output()` 自动清除该命令，恢复正常控制。

---

## 7. 日志层：周期性状态输出

### 7.1 [app] 日志（2Hz）

**文件**：`src/main.c`

```c
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);
```

`main()` 的主循环每 500ms 打印一次：

```c
LOG_INF("pitch %.2f roll %.2f yaw %.1f gy %.1f gz %.1f current %d/%d "
        "height %d batt %.2f V",
        status.pitch_deg, status.roll_deg, status.yaw_deg,
        last_imu_sample.gy_dps, last_imu_sample.gz_dps,
        status.left_wheel_current, status.right_wheel_current,
        status.height, battery.voltage_v);
```

输出示例：
```
[app] pitch -0.80 roll -1.03 yaw -0.2 gy 0.1 gz -0.1 current 0/0 height 38 batt 23.20 V
```

### 7.2 [ascento] 日志（~1Hz）

**文件**：`src/ascento_balance.c`

```c
LOG_MODULE_REGISTER(ascento, LOG_LEVEL_INF);
```

三种状态日志：

| 状态 | 频率 | 级别 | 字段 |
|------|------|------|------|
| 模型未就绪 | ~1024 tick | `LOG_WRN` | `params_ready`, `enable_req`, `wheel_fb_ok`, `calib` |
| fault 保护 | ~1024 tick | `LOG_WRN` | `pitch`, `theta_eq`, `err_deg`, `recover` |
| 正常平衡 | ~1024 tick | `LOG_INF` | `pitch`, `err`, `torque`, `cur`, `L`, `K` |

Flash 操作（save/load/reset）使用 `printk()` 直接输出，前缀为 `ascento:`。

### 7.3 日志输出机制

`CONFIG_LOG_MODE_IMMEDIATE=y` 使 `LOG_INF`/`LOG_WRN`/`LOG_ERR` 在调用者上下文中同步写入 UART，无异步缓冲。因此：

- 日志与 Shell 输入/输出在同一物理串口上交错
- 日志可能在命令输入过程中插入，不影响命令解析（Shell 以换行符分隔命令）
- 日志输出频率受各自 tick 计数器控制，不会淹没串口

---

## 8. 完整调用链

```
物理 UART (PA9/PB7 @ 115200 baud)
    │
    ▼
Zephyr UART 驱动 (CONFIG_SERIAL=y, CONFIG_UART_CONSOLE=y)
    │
    ├──► Zephyr Shell 线程 (CONFIG_SHELL=y, zephyr,shell-uart = &usart1)
    │        │
    │        ├─ 解析输入行，匹配已注册命令
    │        │
    │        ├─ "robot" 命令 ──► cmd_robot_*() ──► control_*() API
    │        │        │                    (pid_balance_control.c, k_mutex 保护)
    │        │        ├─ control_set_enable()
    │        │        ├─ control_set_height()
    │        │        ├─ control_set_joystick()
    │        │        ├─ control_get_status()
    │        │        └─ ...
    │        │
    │        └─ "motor" 命令 ──► cmd_motor_*() ──► motor_debug_*() API
    │                  │                    (motor_debug.c, k_spinlock 保护)
    │                  ├─ enter_motor_debug_mode()  [关闭平衡控制]
    │                  ├─ motor_debug_set_wheel_current()
    │                  ├─ motor_debug_dm_enable()
    │                  ├─ motor_debug_set_dm_mit()
    │                  └─ ...
    │
    └──► Zephyr 日志后端 (CONFIG_LOG=y, CONFIG_LOG_MODE_IMMEDIATE=y)
              │
              ├─ [app] 模块 (main.c, LOG_MODULE_REGISTER(app))
              │        500ms 周期：pitch/roll/yaw/current/battery
              │
              └─ [ascento] 模块 (ascento_balance.c, LOG_MODULE_REGISTER(ascento))
                       诊断：model-blocked / faulted / active gains
```

### 关键文件索引

| 文件 | 职责 |
|------|------|
| `boards/arm/dji_f407igh6_c/dji_f407igh6_c.dts` | 引脚复用、UART 配置、chosen 节点 |
| `prj.conf` | Kconfig：SERIAL, CONSOLE, SHELL, LOG |
| `src/shell_commands.c` | 所有 Shell 命令处理器 + SHELL_CMD_REGISTER |
| `src/pid_balance_control.c` | 活跃的 `control_*` API（PID 平衡控制器） |
| `src/motor_debug.c` | `motor_debug_*` API（直接电机控制） |
| `src/main.c` | 启动序列、控制线程、`[app]` 周期日志 |
| `src/ascento_balance.c` | Ascento 平衡模型、`[ascento]` 诊断日志 |
| `src/control.h` | 共享类型（`control_status_t`, `control_output_t`, `robot_motion_t`） |
| `src/motor_debug.h` | 共享类型（`motor_debug_output_t`, `motor_debug_dm_command_t`） |
| `CMakeLists.txt` | 构建系统——确认编译 `pid_balance_control.c`（非 `control.c`） |

---

## 9. 串口命令速查

> 完整命令语法和参数说明见 [SERIAL_COMMAND_REFERENCE.md](SERIAL_COMMAND_REFERENCE.md)

### 9.1 启动检查

```text
robot status                     # 看固件版本/状态
motor can status all             # CAN 总线是否正常
motor wheel status all           # 轮电机反馈是否在线
motor dm status all              # 关节电机反馈是否在线
```

### 9.2 平衡控制

```text
robot enable 1                   # 开启平衡控制
robot status                     # 确认 enable=1
robot enable 0                   # 关闭平衡控制
```

### 9.3 实时调参

```text
robot param                      # 查看所有参数
robot param list                 # 列出可调参数名
robot param <name> <value>       # 设置参数（立即生效，断电丢失）
robot param save                 # 保存到 Flash（永久）
robot param reset                # 恢复代码默认值
```

### 9.4 电机调试

```text
motor wheel current left 100 3000    # 左轮 100mA, 3s
motor wheel pair 100 -100 3000       # 原地旋转
motor dm pos left 2.381 1.0 3000     # 左关节位置控制
motor dm nudge left 0.05             # 左关节微动 +0.05 rad
motor dm diag left                   # 左关节完整诊断
```

### 9.5 安全停止

```text
robot enable 0                   # 关闭平衡控制
motor debug stop                 # 停止所有手动调试
motor wheel stop                 # 轮电流归零
motor dm stop left               # 左关节失力
motor dm stop right              # 右关节失力
```

### 9.6 CAN 总线恢复

```text
motor can status all             # 查看 CAN 状态
motor can recover all            # bus-off 恢复
```

---

## 10. 使用注意事项

### 10.1 robot enable 超时

`robot enable 1` 有 **700ms 超时**。发送后应立即 `robot status` 确认 `enable=1`。如果仍为 0，命令已老化需重发。

### 10.2 调试模式互斥

任何 `motor wheel/dm` 调试命令执行时，会自动调用 `enter_motor_debug_mode()` 关闭平衡控制器。用完后**必须 `motor debug stop`** 恢复，否则 `robot enable 1` 不会生效。

### 10.3 关节无力恢复

关节无力时，按复位键重启比手动 `motor dm enable` 更可靠，会走完整使能流程。

### 10.4 安全停止序列

```text
robot enable 0 → motor debug stop → motor wheel stop → motor dm stop left/right
```

### 10.5 三层诊断法

排查顺序：**模型内部 → 门状态 → 电机实际**，跳过任一层都会导致误判。

| 数据层 | 来源 | 频率 | 看什么 |
|--------|------|------|--------|
| 模型内部 | `[ascento]` 日志 `cur=` | ~1 Hz | 模型算了什么电流 |
| 控制器状态 | `robot status` `enable/wheels/fault` | 按需 | 哪个门关掉了 |
| 电机实际 | `motor wheel/dm status` `cmd` vs `motor_current` | 按需 | 反馈是否在线 |

### 10.6 轮子不转快速排查

```
[ascento] 日志 cur 非零 → 模型有输出
    │
    ▼
robot status current=(0,0) → 门关掉了
    │                        查 enable= / wheels= / fault=
    ▼
motor wheel status cmd 非零, motor_current≈0 → VESC 未执行
    │                                          检查 VESC 电源/CAN2 接线
    ▼
motor wheel status cmd 非零, motor_current 非零 → 机械卡死
```

### 10.7 CAN bus-off

CAN 进入 `bus-off` 状态时用 `motor can recover` 恢复，同时检查终端电阻和接线。

### 10.8 日志与命令共存

`CONFIG_LOG_MODE_IMMEDIATE=y` 使日志同步输出，会在命令输入过程中插入。Shell 以换行符分隔命令，日志不会影响命令解析，但可能干扰视觉。如需安静环境，可在日志打印间隔内操作。

---

## 11. 关联文档

| 文档 | 内容 |
|------|------|
| [SERIAL_COMMAND_REFERENCE.md](SERIAL_COMMAND_REFERENCE.md) | 所有串口命令的完整语法、参数和示例 |
| [SERIAL_README_ZH.md](SERIAL_README_ZH.md) | 串口终端连接和 picocom 使用 |
| [PARAM_TUNING_GUIDE.md](PARAM_TUNING_GUIDE.md) | 参数调优指南 |
| [PERSISTENT_PARAMS_GUIDE.md](PERSISTENT_PARAMS_GUIDE.md) | 参数持久化（Flash NVS）说明 |
| [ASCENTO_WHEEL_NOT_ROTATE_DEBUG_MANUAL_ZH.md](ASCENTO_WHEEL_NOT_ROTATE_DEBUG_MANUAL_ZH.md) | 轮子不转排查手册 |
| [MOTOR_DEBUG_README_ZH.md](MOTOR_DEBUG_README_ZH.md) | 电机调试模块详细说明 |
