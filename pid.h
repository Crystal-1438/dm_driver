#ifndef PID_H_
#define PID_H_

// 外部库
#include "math.h"
#include "stdint.h"
// HAL层
#include "common.h"

typedef enum PID_Mode_e {
	PID_POSITION = 0,	// 位置式
	PID_DELTA,			// 增量式
	PID_COMP_POSITION,	// PI分离
} PID_Mode;

#pragma pack(1)
typedef struct PID_config_t {
    float KP;  // 比例量，在PI分离模式中为粗调的kp
    float KI;
    float KD;
    float KP_fine;      // PI分离模式中精调的kp
    float range_rough;  // PI分离模式中粗调阈值
    float range_fine;   // PI分离模式中精调阈值
	float error_max;	// error_sum的最大值
	float outputMax;
    float compensation;  // PI分离模式中，当误差介于粗调范围与精调范围时的精调kp补偿值
	float error_preload;

    float Irange;   // 积分范围，误差超出此范围时积分置零
    PID_Mode PID_mode;
} PID_Config;

typedef struct PID_t {
    PID_Config config;
    float error[3];
    float d_error;  // 微分值
    float error_sum;
    float fdb;
    float ref;
    float output;
    float output_unlimited;  // 经outputMax限制前的原始输出
    float error_delta;
} PID;
#pragma pack()

void PID_Init(PID* pid, PID_Config* config);
void PID_Calc(PID* pid);
void PID_SetConfig_Pos(PID_Config* obj, float kp, float ki, float kd, float errormax, float outputmax);
void PID_SetConfig_Comp(PID_Config* obj, float kp_rough, float kp_fine, float ki, float kd, float rangerough, float rangefine, float compensation, float errorpreload, float errormax,
                        float outputmax);
void PID_ChangeConfig_Pos(struct PID_config_t* obj, float kp, float ki, float kd, float errormax);
#endif
