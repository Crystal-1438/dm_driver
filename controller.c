#include <controller.h>

// 读各种算法的fdb和ref请直接读controller结构体里面的总fdb和ref，不要读子结构体里面的ref和fdb
// 原因在于，子结构体的ref和fdb高频参与计算，即高频读取，而controller里面的总ref和fdb读写频率相对较低
// 由于受到硬件读写频率的限制，读子结构体的ref和fdb容易出各种bug，而controller的总ref和fdb相对较稳定

void controller_calc(controller* obj) {
    // 单级位置环控制
    if (obj->config.control_depth == POS_CONTROL_SINGLE) {
        if (obj->config.control_type == PID_MODEL) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
            if (obj->pid_pos_data.config.D_ahead == D_AHEAD_USED) {
                obj->pid_pos_data.d_error = -(obj->fdb_speed - obj->speed_comp);
            }
#pragma GCC diagnostic pop
            obj->pid_pos_data.fdb = obj->fdb_position;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将controller结构体的ref经td滤波后再赋值给pid结构体的ref
                obj->td_data.ref = obj->ref_position;
                TDFunction_independent(&obj->td_data);
                obj->pid_pos_data.ref = obj->td_data.v1;
            } else {
                obj->pid_pos_data.ref = obj->ref_position;
            }
            PID_Calc(&obj->pid_pos_data);
            obj->output = obj->pid_pos_data.output;  // 将位置环PID计算出的输出直接作为输出电流
            obj->output += obj->output_comp;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将td滤波后的微分信号补偿给output
                obj->output += obj->td_data.config.J * obj->td_data.v2; 
            }
            if (obj->config.eso_status == ESO_USED) {  // 若使用eso，则观测output与fdb，并对output进行补偿
                obj->eso_data.output = _MID(obj->output, -obj->pid_pos_data.config.outputMax, obj->pid_pos_data.config.outputMax);
                obj->eso_data.fdb = obj->fdb_speed;
                ESOFunction_independent(&obj->eso_data);    
                obj->output -= obj->eso_data.config.output_mode * obj->eso_data.z3 / obj->eso_data.config.b;
                obj->eso_data.output = obj->output;
            }
            obj->output = _MID(obj->output, -obj->pid_pos_data.config.outputMax, obj->pid_pos_data.config.outputMax);
        }
        else if(obj->config.control_type == lqr_model){
            obj->lqr_data.x1 = obj->fdb_position * DEG2RAD;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将controller结构体的ref经td滤波后再赋值给pid结构体的ref
                obj->td_data.ref = obj->ref_position;
                TDFunction_independent(&obj->td_data);
                obj->lqr_data.x1_ref = obj->td_data.v1 * DEG2RAD;
            } else {
                obj->lqr_data.x1_ref = obj->ref_position * DEG2RAD;
            } 
                obj->lqr_data.x2 = obj->fdb_speed * DEG2RAD;
            LQR_Calc(&obj->lqr_data);
            obj->output = obj->lqr_data.output;            
            obj->output += obj->output_comp;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将td滤波后的微分信号补偿给output
                obj->output += obj->td_data.config.J * obj->td_data.v2; 
            }
            if (obj->config.eso_status == ESO_USED) {  // 若使用eso，则观测output与fdb，并对output进行补偿
                // obj->eso_data.output = _MID(obj->output, -obj->pid_pos_data.config.outputMax, obj->pid_pos_data.config.outputMax);
                obj->eso_data.fdb = obj->fdb_speed;
                ESOFunction_independent(&obj->eso_data);    
                obj->output -= obj->eso_data.config.output_mode * obj->eso_data.z2 / obj->eso_data.config.b;
                obj->eso_data.output = obj->output;
            }
            obj->output = _MID(obj->output, -obj->lqr_data.config.outputmax, obj->lqr_data.config.outputmax);
        }
        // 串级控制
    } else if (obj->config.control_depth == POS_CONTROL_CASCADE) {
        if (obj->config.control_type == PID_MODEL) {
            obj->pid_pos_data.fdb = obj->fdb_position;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将controller结构体的ref经td滤波后再赋值给pid结构体的ref
                obj->td_data.ref = obj->ref_position;
                TDFunction_independent(&obj->td_data);
                obj->pid_pos_data.ref = obj->td_data.v1;
            } else {
                obj->pid_pos_data.ref = obj->ref_position;
            }            
            PID_Calc(&obj->pid_pos_data);
            obj->ref_speed = obj->pid_pos_data.output + obj->speed_comp;  // 将位置环PID计算出的输出加上位置环补偿后作为速度环目标值
            }
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将td滤波后的微分信号补偿给ref_speed
                obj->ref_speed += obj->td_data.v2;
            }
            obj->pid_speed_data.fdb = obj->fdb_speed;
            obj->pid_speed_data.ref = obj->ref_speed;
            
            PID_Calc(&obj->pid_speed_data);
            obj->output = obj->pid_speed_data.output;  // 将速度环PID计算出的输出作为输出电流
            obj->output += obj->output_comp;
            if (obj->config.eso_status == ESO_USED) {  // 若使用eso，则观测output与fdb，并对output进行补偿
                obj->eso_data.output = _MID(obj->output, -obj->pid_speed_data.config.outputMax, obj->pid_speed_data.config.outputMax);
                obj->eso_data.fdb = obj->pid_speed_data.fdb;
                ESOFunction_independent(&obj->eso_data);
                obj->output -= obj->eso_data.config.output_mode * obj->eso_data.z2 / obj->eso_data.config.b;
            }
            obj->output = _MID(obj->output, -obj->pid_speed_data.config.outputMax, obj->pid_speed_data.config.outputMax);
        }
        // 单级速度环控制
    else if (obj->config.control_depth == SPEED_CONTROL) {
        if (obj->config.control_type == PID_MODEL) {
            obj->pid_speed_data.fdb = obj->fdb_speed;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将controller结构体的ref经td滤波后再赋值给pid结构体的ref
                obj->td_data.ref = obj->ref_speed;
                TDFunction_independent(&obj->td_data);
                obj->pid_speed_data.ref = obj->td_data.v1;
            } else {
                obj->pid_speed_data.ref = obj->ref_speed;
            }
            PID_Calc(&obj->pid_speed_data);
            obj->output = obj->pid_speed_data.output;
            obj->output += obj->output_comp;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将td滤波后的微分信号补偿给output
                obj->output += obj->td_data.config.J * obj->td_data.v2;
            }
            if (obj->config.eso_status == ESO_USED) {  // 若使用eso，则观测output与fdb，并对output进行补偿
                obj->eso_data.output = _MID(obj->output, -obj->pid_speed_data.config.outputMax, obj->pid_speed_data.config.outputMax);
                obj->eso_data.fdb = obj->pid_speed_data.fdb;
                ESOFunction_independent(&obj->eso_data);
                obj->output -= obj->eso_data.z2 / obj->eso_data.config.b;
            }
            obj->output = _MID(obj->output, -obj->pid_speed_data.config.outputMax, obj->pid_speed_data.config.outputMax);
        } else if (obj->config.control_type == SMC_MODEL) {
            obj->smc_speed_data.fdb = obj->fdb_speed;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将controller结构体的ref经td滤波后再赋值给pid结构体的ref
                obj->td_data.ref = obj->ref_speed;
                TDFunction_independent(&obj->td_data);
                obj->smc_speed_data.ref = obj->td_data.v1;
            } else {
                obj->smc_speed_data.ref = obj->ref_speed;
            }
            SMC_Calc(&obj->smc_speed_data);
            obj->output = obj->smc_speed_data.output;
            obj->output += obj->output_comp;
            if (obj->config.td_status == TD_USED) {  // 若使用td，则将td滤波后的微分信号补偿给output
                obj->output += obj->td_data.config.J * obj->td_data.v2;
            }
            if (obj->config.eso_status == ESO_USED) {  // 若使用eso，则观测output与fdb，并对output进行补偿
                obj->eso_data.output = _MID(obj->output, -obj->smc_speed_data.config.outputMax, obj->smc_speed_data.config.outputMax);
                obj->eso_data.fdb = obj->smc_speed_data.fdb;
                ESOFunction_independent(&obj->eso_data);
                obj->output -= obj->eso_data.config.output_mode * obj->eso_data.z2 / obj->eso_data.config.b;
            }
            obj->output = _MID(obj->output, -obj->smc_speed_data.config.outputMax, obj->smc_speed_data.config.outputMax);
        }
    }
}

// 务必确保传入的config结构体已经memset置零!!!
controller* create_controller(controller_config* _config) {
    controller* obj = RT_MALLOC(sizeof(controller));
    memset(obj, 0, sizeof(controller));
    obj->config = *_config;
    if (obj->config.control_type == PID_MODEL) {
        PID_Init(&obj->pid_pos_data, &obj->config.position_pid_config);
        PID_Init(&obj->pid_speed_data, &obj->config.speed_pid_config);
    }
    if (obj->config.control_type == SMC_MODEL) {
        SMC_Init(&obj->smc_speed_data, &obj->config.speed_smc_config);
    }
    if (obj->config.control_type == lqr_model){
        LQR_Init(&obj->lqr_data, &obj->config.lqr_config);
    }
    if (obj->config.td_config.r > 1) {
        obj->config.td_status = TD_USED;
        TD_Init(&obj->td_data, &obj->config.td_config);
    }
    if (obj->config.eso_config.beta1 > 1) {
        obj->config.eso_status = ESO_USED;
        ESO_Init(&obj->eso_data, &obj->config.eso_config);
    }
    return obj;
}

void set_ref_pos(controller* con, float ref) {
    con->ref_position = ref;
}

void set_ref_speed(controller* con, float ref) {
    con->ref_speed = ref;
}