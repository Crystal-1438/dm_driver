/**
 ******************************************************************************
 * 文件名        : pid.c
 * 文件描述      ：PID控制算法
 * 创建时间      ：2019.11.9
 * 作者          ：刘文熠
 *-----------------------------------------------------------------------------
 * 最近修改时间  ：2025.9.6
 * 修改人        ：赵苏亚
 * 内容          ：加入模糊PID模式
 ******************************************************************************
 * 1.本代码基于STMF427IIT6开发，编译环境为Keil 5，基于FreeRTOS进行开发。
 * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
 * 3.本代码包含大量中文注释，请以UTF-8编码格式打开
 * 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
 *
 * Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
 ******************************************************************************
 */

#include "pid.h"
#include <math.h>

const double EPS = 1e-6;

// ----------- 模糊PID相关参数与函数定义 -----------
// 模糊规则表
static const float FUZZY_KP_TABLE_NORMAL[FUZZY_E_LEVEL][FUZZY_DE_LEVEL] = {
    {0.3, 0.3, 0.4, 0.5, 0.6, 0.7, 0.7},
    {0.3, 0.4, 0.5, 0.6, 0.7, 0.7, 0.7},
    {0.4, 0.5, 0.6, 0.7, 0.7, 0.7, 0.7},
    {0.5, 0.6, 0.7, 0.8, 0.7, 0.6, 0.5},
    {0.7, 0.7, 0.7, 0.7, 0.6, 0.5, 0.4},
    {0.7, 0.7, 0.7, 0.6, 0.5, 0.4, 0.3},
    {0.7, 0.7, 0.6, 0.5, 0.4, 0.3, 0.3}
};
static const float FUZZY_KI_TABLE_NORMAL[FUZZY_E_LEVEL][FUZZY_DE_LEVEL] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0},
    {0.0, 0.0, 0.5, 1.0, 0.5, 0.0, 0.0},
    {0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};
static const float FUZZY_KD_TABLE_NORMAL[FUZZY_E_LEVEL][FUZZY_DE_LEVEL] = {
    {0.8, 0.8, 0.9, 1.0, 0.9, 0.8, 0.8},
    {0.8, 0.7, 0.8, 0.9, 0.8, 0.7, 0.8},
    {0.9, 0.8, 0.6, 0.5, 0.6, 0.8, 0.9},
    {1.0, 0.9, 0.5, 0.1, 0.5, 0.9, 1.0},
    {0.9, 0.8, 0.6, 0.5, 0.6, 0.8, 0.9},
    {0.8, 0.7, 0.8, 0.9, 0.8, 0.7, 0.8},
    {0.8, 0.8, 0.9, 1.0, 0.9, 0.8, 0.8}
};

// 模糊量化函数，将输入值归一化到[-3,3]区间
static int fuzzy_quantize(float x) {
	if (x <= -3.0f) return 0;
	if (x >= 3.0f) return 6;
	return (int)(x + 3.5f);
}

void PID_Init(PID* pid, PID_Config* config) { pid->config = *config; }

/**
 * @brief PID计算函数，多种模式合在一起
 * @param PID结构体
 */
void PID_Calc(PID* pid) {
    pid->error[2] = pid->error[1];        // 上上次误差
    pid->error[1] = pid->error[0];        // 上次误差
    pid->error[0] = pid->ref - pid->fdb;  // 本次误差
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (pid->config.D_ahead != D_AHEAD_USED) {
        if (fabsf(pid->error[0] - pid->error[1]) > (float)EPS) {
            pid->d_error = pid->error[0] - pid->error[1];
        } else if (fabsf(pid->error[0] - pid->error[1]) <= (float)EPS && fabsf(pid->error[1] - pid->error[0]) <= (float)EPS && fabsf(pid->d_error) < (1000.0f * (float)EPS)) {
            pid->d_error = pid->error[0] - pid->error[1];
        }
    }
#pragma GCC diagnostic pop

    // 位置式PID
    if (pid->config.PID_mode == PID_POSITION) {
        if (fabsf(pid->error[0]) < pid->config.Irange) {
            pid->error_sum += pid->error[0];
            pid->error_sum = _MID(pid->error_sum, -pid->config.error_max, pid->config.error_max); // 积分限幅
            pid->output = pid->config.KP * pid->error[0] + pid->config.KI * pid->error_sum + pid->config.KD * pid->d_error;
        } else {
            pid->error_sum = 0;
            pid->output = pid->config.KP * pid->error[0] + pid->config.KD * pid->d_error;
        }
    }
    // 增量式PID
    else if (pid->config.PID_mode == PID_DELTA) {
        pid->output += pid->config.KP * (pid->error[0] - pid->error[1]) + pid->config.KD * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) + pid->config.KI * pid->error[0];
    }
    // PI分离模式，用于摩擦轮和拨弹
    else if (pid->config.PID_mode == PID_COMP_POSITION) {
        pid->error_delta = pid->error[0] - pid->error[1];
        if (fabsf(pid->error[0]) > pid->config.range_rough) {  // 本次误差大于粗调阈值
            pid->output = fsgn(pid->error[0]) * pid->config.outputMax;        // 输出拉满
            pid->error_sum = 0;
        } else if (fabsf(pid->error[0]) < pid->config.range_fine) {  // 本次误差小于细调阈值
            pid->error_sum += pid->error[0];                     
            pid->error_sum = _MID(pid->error_sum, -pid->config.error_max, pid->config.error_max); // 积分限幅
            pid->output = pid->config.KP_fine * pid->error[0] + pid->config.KI * pid->error_sum + pid->config.KD * pid->error_delta;
        } else {  // 误差介于粗调、精调区间之间时，通过位置环补偿改变误差，通过积分预载给定积分
            pid->output = pid->config.KP * (pid->error[0] + fsgn(pid->error[0]) * pid->config.compensation) + pid->config.KD * pid->error_delta;
            pid->error_sum = fsgn(pid->error[0]) * pid->config.error_preload;
        }
    }
    // 模糊PID
    else if (pid->config.PID_mode == PID_FUZZY) {
		// 归一化误差和误差变化率
		float e_norm = pid->error[0] / pid->config.error_domain * 3.0f;
		float de_norm = pid->d_error / pid->config.d_error_domain * 3.0f;
		int e_idx = fuzzy_quantize(e_norm);
		int de_idx = fuzzy_quantize(de_norm);
		// 查表获得增益
		float kp = pid->config.KP * (*pid->config.fuzzy_table[0])[e_idx][de_idx];
		float ki = pid->config.KI * (*pid->config.fuzzy_table[1])[e_idx][de_idx];
		float kd = pid->config.KD * (*pid->config.fuzzy_table[2])[e_idx][de_idx];
		// 基于位置式PID
		if (fabsf(pid->error[0]) < pid->config.Irange) {
			pid->error_sum += pid->error[0];
			pid->error_sum = _MID(pid->error_sum, -pid->config.error_max,
								  pid->config.error_max);
			pid->output = kp * pid->error[0] + ki * pid->error_sum +
						  kd * (pid->error[0] - pid->error[1]);
		} else {
			pid->error_sum = 0;
			pid->output =
				kp * pid->error[0] + kd * (pid->error[0] - pid->error[1]);
		}
	}

    /*------- 输出上限 -------*/
    pid->output_unlimited = pid->output;
    pid->output = _MID(pid->output, -pid->config.outputMax, pid->config.outputMax);
}

// 配置位置式PID的参数
void PID_SetConfig_Pos(PID_Config* obj, float kp, float ki, float kd, float errormax, float outputmax) {
    obj->PID_mode = PID_POSITION;
    obj->KP = kp;
    obj->KI = ki;
    obj->KD = kd;
    obj->error_max = errormax;
    obj->outputMax = outputmax;
    obj->Irange = 0;
    obj->D_ahead = D_AHEAD_UNUSED;  // 默认关闭微分先行
}

/**
 * @brief 修改位置式PID的参数，用于不同运行模式修改电机PID参数
 * @note 只修改 KP、KI、KD、errormax
 */ 
void PID_ChangeConfig_Pos(struct PID_config_t *obj, float kp, float ki, float kd, float errormax) {
    obj->KP = kp;
    obj->KI = ki;
    obj->KD = kd;
    obj->error_max = errormax;
}

// 配置PI分离式PID的参数
void PID_SetConfig_Comp(PID_Config* obj, float kp_rough, float kp_fine, float ki, float kd, float rangerough, float rangefine, float compensation, float errorpreload, float errormax,
                        float outputmax) {
    obj->PID_mode = PID_COMP_POSITION;
    obj->KP = kp_rough;      // 粗调KP
    obj->KP_fine = kp_fine;  // 细调KP
    obj->KI = ki;
    obj->KD = kd;
    obj->range_rough = rangerough;      // 粗调区间
    obj->range_fine = rangefine;        // 细调区间
    obj->compensation = compensation;   // 位置环补偿
    obj->error_preload = errorpreload;  // 位置环积分预载
    obj->error_max = errormax;
    obj->outputMax = outputmax;
    obj->D_ahead = D_AHEAD_UNUSED;  // 默认关闭微分先行
}

// 配置模糊PID的参数
void PID_SetConfig_Fuzzy(PID_Config* obj, float kp, float ki, float kd,float error_domain,float d_error_domain, float errormax, float outputmax) {
	obj->PID_mode = PID_FUZZY;
	obj->KP = kp;
	obj->KI = ki;
	obj->KD = kd;
	obj->error_max = errormax;
	obj->outputMax = outputmax;
	obj->Irange = 0;
	obj->D_ahead = D_AHEAD_UNUSED;	// 默认关闭微分先行
	obj->error_domain = fabsf(error_domain);
	obj->d_error_domain = fabsf(d_error_domain);
	//默认使用普通模糊规则表
	obj->fuzzy_table[0] =
		(float(*)[FUZZY_E_LEVEL][FUZZY_DE_LEVEL])FUZZY_KP_TABLE_NORMAL;
	obj->fuzzy_table[1] =
		(float(*)[FUZZY_E_LEVEL][FUZZY_DE_LEVEL])FUZZY_KI_TABLE_NORMAL;
	obj->fuzzy_table[2] =
		(float(*)[FUZZY_E_LEVEL][FUZZY_DE_LEVEL])FUZZY_KD_TABLE_NORMAL;
}