#include <controller.h>

// 读各种算法的fdb和ref请直接读controller结构体里面的总fdb和ref，不要读子结构体里面的ref和fdb
// 原因在于，子结构体的ref和fdb高频参与计算，即高频读取，而controller里面的总ref和fdb读写频率相对较低
// 由于受到硬件读写频率的限制，读子结构体的ref和fdb容易出各种bug，而controller的总ref和fdb相对较稳定

void controller_calc(controller* obj) {
    // 单级位置环控制
    if (obj->config.control_depth == POS_CONTROL_SINGLE) {
        obj->pid_pos_data.fdb = obj->fdb_position;
        obj->pid_pos_data.ref = obj->ref_position;
        PID_Calc(&obj->pid_pos_data);

        obj->output = obj->pid_pos_data.output;  // 将位置环PID计算出的输出直接作为输出电流
        obj->output += obj->output_comp;
        obj->output = _MID(obj->output, -obj->pid_pos_data.config.outputMax, obj->pid_pos_data.config.outputMax);

        // 串级控制
    } else if (obj->config.control_depth == POS_CONTROL_CASCADE) {
        obj->pid_pos_data.fdb = obj->fdb_position;
        obj->pid_pos_data.ref = obj->ref_position;
        PID_Calc(&obj->pid_pos_data);
        obj->ref_speed = obj->pid_pos_data.output + obj->speed_comp;  // 将位置环PID计算出的输出加上位置环补偿后作为速度环目标值

        obj->pid_speed_data.fdb = obj->fdb_speed;
        obj->pid_speed_data.ref = obj->ref_speed;
        PID_Calc(&obj->pid_speed_data);

        obj->output = obj->pid_speed_data.output;  // 将速度环PID计算出的输出作为输出电流
        obj->output += obj->output_comp;
        obj->output = _MID(obj->output, -obj->pid_speed_data.config.outputMax, obj->pid_speed_data.config.outputMax);

        // 单级速度环控制
    } else if (obj->config.control_depth == SPEED_CONTROL) {
        obj->pid_speed_data.fdb = obj->fdb_speed;
        obj->pid_speed_data.ref = obj->ref_speed;
        PID_Calc(&obj->pid_speed_data);

        obj->output = obj->pid_speed_data.output;
        obj->output += obj->output_comp;
        obj->output = _MID(obj->output, -obj->pid_speed_data.config.outputMax, obj->pid_speed_data.config.outputMax);
    }
}

// 务必确保传入的config结构体已经memset置零!!!
controller* create_controller(controller_config* _config) {
    controller* obj = RT_MALLOC(sizeof(controller));
    memset(obj, 0, sizeof(controller));
    obj->config = *_config;
    // 只有 PID 一种算法，两个环的参数一并初始化。
    // 单速度环时 position_pid_config 为全零，初始化了也不会被 controller_calc 用到。
    PID_Init(&obj->pid_pos_data, &obj->config.position_pid_config);
    PID_Init(&obj->pid_speed_data, &obj->config.speed_pid_config);
    return obj;
}

void set_ref_pos(controller* con, float ref) {
    con->ref_position = ref;
}

void set_ref_speed(controller* con, float ref) {
    con->ref_speed = ref;
}
