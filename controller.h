#ifndef _CONTROLLER_H
#define _CONTROLLER_H

// 外部库
#include <stdlib.h>
#include <string.h>
// HAL层
#include "common.h"
#include "pid.h"

/* 本控制器只保留 PID 算法，三种环路拓扑：
 *     SPEED_CONTROL        单速度环
 *     POS_CONTROL_SINGLE   单位置环
 *     POS_CONTROL_CASCADE  位置-速度串级环
 *
 * 已移除：SMC(滑模)、ADRC(TD/ESO)、MRAC、LQR。
 * 对应地不再包含 smc.h / adrc.h / mrac.h / lqr.h。
 */

/* 仅剩 PID 一种。保留该枚举是为了让既有的
 *     cfg.control_type = PID_MODEL;
 * 这类配置代码不用改动就能继续编译。 */
enum controller_basic_type_e { PID_MODEL = 0 };

enum controller_depth_e { SPEED_CONTROL = 0,
                          POS_CONTROL_CASCADE,
                          POS_CONTROL_SINGLE };

#pragma pack(1)
typedef struct Controller_Config_t {
    enum controller_basic_type_e control_type;  // 基础控制器算法类型，现仅 PID_MODEL
    enum controller_depth_e control_depth;      // 控制器深度
    // 双环PID配置结构体
    PID_Config speed_pid_config;
    PID_Config position_pid_config;
} controller_config;

typedef struct controller_t {
    controller_config config;
    PID pid_speed_data;
    PID pid_pos_data;
    float output;
    float ref_speed;
    float ref_position;
    float fdb_speed;
    float fdb_position;
    float output_comp;  // 输出前馈
    float speed_comp;   // 速度前馈
} controller;
#pragma pack()

void controller_calc(controller* obj);
controller* create_controller(controller_config* _config);

void set_ref_pos(controller* con, float ref);
void set_ref_speed(controller* con, float ref);
#endif
