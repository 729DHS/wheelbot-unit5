/**
 * @file protocol_service.h
 * @brief PC 控制协议服务 — USART1 文本协议 (ISR + msgq + 工作线程)
 *
 * 协议格式: CMD:arg1:arg2\n (文本行, 人可直接手打, 脚本也可用)
 *
 * USART1 (PA9 TX / PB7 RX, 115200) 专用于上位机通信,
 * 与 USART6 Shell/CSV 独立, 互不干扰。
 *
 * ISR 收字节 → 换行触达帧 → k_msgq 投递 → 工作线程消费分发。
 *
 * 支持命令:
 *   MOVE:<h_mm>:<phi_deg>   点到点移动
 *   JOG_H:<delta_mm>        h 向微调
 *   JOG_PHI:<delta_deg>     phi 向微调
 *   STOP                    紧急停止
 *   STATUS                  查询状态
 *   STREAM:ON|OFF           连续遥测开关
 *   MODE:DRAG|HOLD          控制模式切换
 *
 * 响应:
 *   OK:CMD:...              成功
 *   ERR:CODE:...            失败
 *   T:...                   遥测流 (STREAM:ON 时)
 */

#ifndef PROTOCOL_SERVICE_H_
#define PROTOCOL_SERVICE_H_

#include <stdint.h>
#include <stdbool.h>

/** @brief 最大帧长度 (含 \n 和 \0) */
#define PROTO_FRAME_MAX 80

/** @brief 消息队列容量 */
#define PROTO_MSGQ_DEPTH 8

/**
 * @brief 初始化协议服务 (打开 USART1, 启动 ISR 和工作线程)
 * @return 0 成功, 负值错误码
 */
int proto_init(void);

/**
 * @brief 发送一行响应
 * @param fmt  格式化字符串
 * @param ...  参数
 */
void proto_send(const char *fmt, ...);

/**
 * @brief 协议流式遥测是否已启用
 */
bool proto_stream_enabled(void);

/**
 * @brief 发送一行遥测 (主循环调用, ~25Hz)
 *
 * 仅当 stream_on 为 true 时输出。
 * 格式: T:ts,h_L,phi_L_deg,h_R,phi_R_deg,m1,m2,m3,m4,t1,t2,t3,t4
 */
void proto_telemetry_tick(void);

#endif /* PROTOCOL_SERVICE_H_ */
