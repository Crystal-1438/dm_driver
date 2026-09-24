#ifndef PID_H_
#define PID_H_

// 外部库
#include "math.h"
#include "stdint.h"
// HAL层
#include "common.h"

// 模糊PID相关参数
#define FUZZY_E_LEVEL 7
#define FUZZY_DE_LEVEL 7

typedef enum PID_Mode_e {
	PID_POSITION = 0,	// 位置式
	PID_DELTA,			// 增量式
	PID_COMP_POSITION,	// PI分离
	PID_FUZZY,			// 模糊PID
} PID_Mode;

typedef enum D_ahead_status_e { 
    D_AHEAD_UNUSED = 0,
    D_AHEAD_USED
} D_ahead_status;  // 是否使用微分先行

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
	float error_domain;	 // 模糊PID中，误差的范围，超出此范围时取表格边界值
	float d_error_domain; // 模糊PID中，误差变化率的范围，超出此范围时取表格边界值
    float (*fuzzy_table[3])[FUZZY_E_LEVEL][FUZZY_DE_LEVEL]; // 模糊PID中，模糊规则表指针数组，按顺序依次为kp、ki、kd

    float Irange;   // 积分范围，误差超出此范围时积分置零
    D_ahead_status D_ahead;  // 微分先行
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
void PID_SetConfig_Fuzzy(PID_Config* obj, float kp, float ki, float kd, float error_domain, float d_error_domain, float errormax, float outputmax);
#endif
