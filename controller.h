#ifndef _CONTROLLER_H
#define _CONTROLLER_H

// 外部库
#include <stdlib.h>
#include <string.h>
// HAL层
#include "adrc.h"
#include "common.h"
#include "mrac.h"
#include "pid.h"
#include "smc.h"
#include "lqr.h"

enum controller_basic_type_e { PID_MODEL = 0,
                               SMC_MODEL,
                               ADRC_MODEL,
                               lqr_model,
};

enum controller_depth_e { SPEED_CONTROL = 0,
                          POS_CONTROL_CASCADE,
                          POS_CONTROL_SINGLE };

enum td_status_e { TD_UNUSED = 0,
                   TD_USED };

enum eso_status_e { ESO_UNUSED = 0,
                    ESO_USED };

#pragma pack(1)
typedef struct Controller_Config_t {
    enum controller_basic_type_e control_type;  // 基础控制器算法类型
    enum controller_depth_e control_depth;      // 控制器深度
    enum td_status_e td_status;                 // 是否使用TD
    enum eso_status_e eso_status;               // 是否使用ESO
    // 使用PID控制时填写双环PID配置结构体
    PID_Config speed_pid_config;
    PID_Config position_pid_config;
    // 使用滑膜控制时配置结构体，目前只使用速度环控制
    Smc_config speed_smc_config;
    // 使用TD时填写配置结构体
    TD_Config td_config;
    // 使用ESO时填写配置结构体
    ESO_Config eso_config; 
    // 新加
    // 使用LQR控制器
    Lqr_config lqr_config;
} controller_config;

typedef struct controller_t {
    controller_config config;
    PID pid_speed_data;
    PID pid_pos_data;
    Smc smc_speed_data;
    TD_t td_data;
    ESO_t eso_data;
    Lqr lqr_data;
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