#pragma once
#include "cvector.h"
#include "bsp_can.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "bsp_delay.h"
#include "monitor.h"
#include "math.h"
#include "board_def.h"
#include "controller.h"
#include "bsp_supervise.h"

/* ARX X5 大关节电机（SDK 内部类名 MotorType4）驱动
 *
 * 用于 X5 的 joint1 / joint2 / joint3，出厂 CAN ID 依次为 0x01 / 0x02 / 0x04（0x03 未使用）。
 * 协议由 libarx_x5_src.so 逆向得到，与达妙(DM)电机**不兼容**，主要差异：
 *   1. 控制帧位域顺序不同：kp 在最前，kd 只占 9 bit，位置挪到中间
 *   2. 反馈帧 ID 等于电机自身 ID（达妙是发到 MST_ID）
 *   3. 错误码在 data[0] 的低 5 位（达妙在高 4 位）
 *   4. 没有 0xFC/0xFD 这类专用使能/失能帧，靠发不同 kd 的控制帧实现
 *   5. 存零点走 0x7FF 广播帧，DLC=4
 *   6. 反馈帧 data[6] 是温度，data[7] 未使用
 */

#pragma pack(1)
struct mt4_motor_t;
typedef void (*control_mt4_func) (struct mt4_motor_t* motor);
typedef float (*calc_fdb_func_mt4) (void* arg);

typedef enum { mt4_stop, mt4_enable } MT4_enable;
typedef enum { mt4_output_normal = 0, mt4_output_reverse } MT4_output_mode;

/* 控制帧位域（DLC=8，共 61 bit，data[0] 高 3 位空闲）
 *   data[0] = kp[11:7]
 *   data[1] = kp[6:0] << 1 | kd[8]
 *   data[2] = kd[7:0]
 *   data[3] = pos[15:8]
 *   data[4] = pos[7:0]
 *   data[5] = vel[11:4]
 *   data[6] = vel[3:0] << 4 | torque[11:8]
 *   data[7] = torque[7:0]
 *
 * 反馈帧位域（DLC=8，can_id == 电机自身 ID）
 *   data[0] & 0x1F      错误码（5 bit，含义未知，见 MT4_ERRCODE_MASK）
 *   data[1..2]          位置 16 bit
 *   data[3] | data[4]hi 速度 12 bit
 *   data[4]lo | data[5] 转矩 12 bit
 *   data[6]             温度，实际值 = (data[6] - 50) / 2  单位 ℃
 *   data[7]             未使用
 */

typedef struct mt4_motor_config_t {
    uint8_t bsp_can_index;
    uint16_t can_tx_id; //发送给电机的id，出厂值 joint1=0x01 joint2=0x02 joint3=0x04
                        //标准帧 ID 最大 0x7FF，故此处比达妙驱动宽一档用 uint16_t
    uint16_t can_rx_id; //电机反馈的id，本协议下**必须等于 can_tx_id**
    controller_config motor_controller_config;
    calc_fdb_func_mt4 pos_fdb_calc;
    calc_fdb_func_mt4 speed_fdb_calc;
    void* arg;
    lost_callback lost_callback_;
    MT4_output_mode output_mode;  //输出方向，默认正转
    //相对频率 :真实频率(控制频率)为 (1000/fre_rel)
    uint16_t fre_rel;

    /* 各物理量(或其绝对值)最大值,用于反馈与控制时线性映射
     * 默认值见 MT4_DEFAULT_MAX_*，由 SDK 逆向得到，勿随意改动 */
    float max_position;  //默认 12.5   rad
    float max_speed;     //默认 18.0   rad/s
    float max_torque;    //默认 30.0   N·m
    float max_kp;        //默认 500.0
    float max_kd;        //默认 50.0   注意线上只有 9 bit，实际可达上限约 6.24

    /* 多圈展开与 2π 软回绕，SDK 行为，见 MT4motor_FeedbackData_Update */
    float position_offset;   //位置偏置，SDK 构造时为 0.0
    float wrap_threshold;    //超出该值即回绕一个 2π，SDK 写死 13.0 rad
} MT4_motor_config;

typedef struct mt4_motor_t {
    MT4_motor_config config;
    MT4_enable enable;

    uint8_t update;
    /* 反馈信息 */
    uint8_t rx_data[8];       //原始数据
    uint8_t errcode;          //错误码，data[0] 的低 5 位。含义未知，达妙那套错误表不适用
    float last_fdb_position;  //上一次位置
    float fdb_position;       //单圈位置（单位：rad）
    float fdb_speed;          //速度 (单位：rad/s)
    float fdb_torque;         //扭矩（单位：N·m）
    int8_t t_mos;             //温度（单位：℃），由 data[6] 换算
    short round;              //电机正向超过 max_position 的次数
    float real_fdb_position;  //多圈展开并回绕后的实际位置（单位：rad）
    int8_t wrap;              //2π 回绕标志，取值 -1 / 0 / +1，控制帧需据此反向补偿

    /* 控制信息 */
    float ref_position;       //位置(rad)
    float ref_speed;          //速度(rad/s)
    float ref_torque;         //转矩(N·m)
    float kp;                 //Kp
    float kd;                 //Kd，注意线上 9 bit，超过约 6.24 会被饱和

    uint8_t (*check_motor_inplace)(struct mt4_motor_t* obj, float range);//电机到位了返回1

    // 频率控制
    uint16_t frequency_cnt;

    monitor_item* monitor;
    FPS_t motor_fps;
    controller* motor_controller;   //控制器
    control_mt4_func control_calc;
} MT4_motor;

#pragma pack()

/* ---- 协议常量 ---- */
#define MT4_CAN_ID_MIN        0x001   // 电机 CAN ID 下限
#define MT4_CAN_ID_MAX        0x7FE   // 上限，0x7FF 被存零点广播帧占用
#define MT4_BROADCAST_ID      0x7FF   // 存零点广播帧 ID
#define MT4_ERRCODE_MASK      0x1F    // 错误码在 data[0] 的低 5 位
#define MT4_KD_WIRE_BITS      9       // kd 在总线上的实际位宽
#define MT4_TEMP_OFFSET       50      // 温度换算：(data[6] - 50) / 2
#define MT4_TEMP_DIV          2
#define MT4_2PI               6.283185307179586f  // M_PI 非 ISO C，自带一份避免工具链差异

#define MT4_DEFAULT_MAX_POSITION   12.5f
#define MT4_DEFAULT_MAX_SPEED      18.0f
#define MT4_DEFAULT_MAX_TORQUE     30.0f
#define MT4_DEFAULT_MAX_KP         500.0f
#define MT4_DEFAULT_MAX_KD         50.0f
#define MT4_DEFAULT_WRAP_THRESHOLD 13.0f

#define MT4_ENABLE_KD              5.0f    // 使能：发 kd=5 的轻阻尼帧
#define MT4_DISABLE_KD             50.0f   // 失能：发最大阻尼帧（SDK 传 100，被钳到 50）

#define MT4_OFFLINE_TIMEOUT_MS     100     // 超过该时长无反馈即判离线（SDK 为 100000 µs）

/* ---- 已实现 ---- */
void       MT4motor_Driver_Init(void);
MT4_motor* MT4motor_Create(MT4_motor_config* config, control_mt4_func func);
void       MT4motor_Calc_Send(void);
void       MT4motor_Enable(MT4_motor* obj);
void       MT4motor_Disable(MT4_motor* obj);
void       MT4motor_Save_Zero(MT4_motor* obj);
void       MT4motor_Reset_Round(MT4_motor* obj);

/* ---- 待实现：协议未知或依赖上位机工具 ---- */

/* 清除错误。
 * SDK 里 MotorType4 没有重写 packClearError()，走的是基类的空实现——
 * 返回一个 can_id=0、DLC=0 的空帧并原样发出去，等于什么都没做。
 * 所以这款电机的清错帧格式**尚未确定**，需要抓包或厂家文档补齐。
 * TODO: 实现 */
void MT4motor_Clear_Err(MT4_motor* obj);

/* 0x7FF 广播命令通用发送接口。
 * 目前只逆向出子命令 0x03（存零点，见 MT4motor_Save_Zero）：
 *     can_id=0x7FF, DLC=4, data = { 0x00, motor_id, 0x00, 0x03 }
 * data[0] 与 data[2] 恒为 0，推测是保留字段或命令组号；
 * 其余子命令（改 ID、改波特率、读参数等）未知。
 * TODO: 实现 */
void MT4motor_Broadcast_Cmd(MT4_motor* obj, uint8_t sub_cmd);

/* 错误码解析。
 * data[0] 低 5 位共 32 种取值，达妙的错误表（0x8 超压 / 0x9 欠压 …）在本协议下不适用，
 * 实际含义需要厂家文档。
 * TODO: 实现 */
const char* MT4motor_Err_String(uint8_t errcode);

/* 在线状态查询。
 * SDK 用「距上次收到反馈的微秒数 > 100000」判定，且是**一次性闩锁**——
 * 置为离线后不会自行恢复。本驱动已有 monitor 机制，此处留接口以便对齐 SDK 语义。
 * TODO: 实现 */
uint8_t MT4motor_Is_Online(MT4_motor* obj);
