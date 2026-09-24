#include "DM_motor.h"
#if _HAL_DMMOTOR_ENABLE 
#include "bsp_log.h"
#include "common.h"
// CAN_ID == DM_motor_config.can_tx_id，即上位机中的 CAN ID
// MST_ID == DM_motor_config.can_rx_id，即上位机中的 Master ID
// 一般而言，CAN ID 和 Master ID 设置为同样值
/* 控制帧ID规则为 
    MIT模式：0x000+CAN_ID 
    位置速度模式：0x100+CAN_ID 
    速度模式：0x200+CAN_ID 
*/
/* 反馈帧ID为
    MST_ID
*/

#define DM_CAN_ID_MIN 0x1  // 为了不掩盖错误码，上位机设置的 CAN_ID 范围应该为 0x1~0xF
#define DM_CAN_ID_MAX 0xF  // 为了不掩盖错误码，上位机设置的 CAN_ID 范围应该为 0x1~0xF
#define DM_MIT_OFFSET 0x000        // MIT模式控制帧ID偏移
#define DM_POS_SPEED_OFFSET 0x100  // 位置速度模式控制帧ID偏移
#define DM_SPEED_OFFSET 0x200      // 速度模式控制帧ID偏移

cvector *dm_motor_list;

uint8_t DMMotor_Check_InPlace(DM_motor *obj , float range) {
    if(obj->monitor->count < 1) return 0;
    return fabs(obj->ref_position - obj->fdb_position) <= range;
}

void DMmotor_FeedbackData_Update(DM_motor *obj, uint8_t *data)
{
    obj->last_fdb_position = obj->fdb_position;
    obj->update = 0;
    obj->monitor->reset(obj->monitor);
    memcpy(obj->rx_data, data, 8);
    obj->errcode = data[0] >> 4;  // 错误码在高4位
    obj->err_status = (DM_err_status)obj->errcode;
    obj->fdb_position = (((int16_t)data[1] << 8) + data[2]) / (float)0x10000 * obj->config.max_position * 2 - obj->config.max_position;
    // uint16_t fdb_speed_temp = ((data[4] & 0xf) << 8) | data[5];

    obj->fdb_speed = (((int16_t)data[3] << 4) + (data[4] >> 4)) / (float)0x1000 * obj->config.max_speed * 2 - obj->config.max_speed;
    uint16_t fdb_torque_temp = ((data[4] & 0x0f) << 8) | data[5];
    obj->fdb_torque = (fdb_torque_temp - 2048) / 2048.0f * (obj->config.max_torque);
    obj->t_mos = data[6];
    obj->t_rotor = data[7];
    if (obj->fdb_position - obj->last_fdb_position > obj->config.max_position)  //这次比上次大很多，即为编码器从小过零导致本次收到反馈的机械角度很大而上次很小，故为往反方向转了一圈
        obj->round--;
    else if (obj->fdb_position - obj->last_fdb_position < -obj->config.max_position)  //这次比上次小很多，即为编码器从大过零导致本次收到的反馈的机械角度很小而上次很大，故为往正方向转了一圈
        obj->round++;
    obj->real_fdb_position = (obj->fdb_position + obj->round * 2* obj->config.max_position);
    FrameRateStatistics(&obj->motor_fps);
}

void DMmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, basic_data_t len) {
    (void)len;
    // DEBUG_ASSERT(len == 8);
    (void)can_id;

    for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
        DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
        if (obj->config.can_rx_id == identifier) {
            DMmotor_FeedbackData_Update(obj, data);
        }
    }
}

void DMmotor_Enable(DM_motor *obj)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Disable(DM_motor *obj)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Clear_Err(DM_motor *obj)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

// TODO: 测试DMmotor_Save_Zero函数
void DMmotor_Save_Zero(DM_motor *obj)
{
    uint8_t data[8] = {0xFF ,0xFF ,0xFF, 0xFF ,0xFF, 0xFF ,0xFF, 0xFE};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Driver_Init(void)
{
    dm_motor_list = cvector_create(sizeof(DM_motor *));
    // BSP_CAN_RegisterRxCallback(0, DMmotor_RxCallBack);
    // BSP_CAN_RegisterRxCallback(1, DMmotor_RxCallBack);
}

DM_motor* DMmotor_Create(DM_motor_config* config , control_dm_func func)
{
    static uint8_t registry[DEVICE_CAN_CNT];
    if (config->bsp_can_index >= DEVICE_CAN_CNT) {// CAN索引超出范围
        printf_log("bsp_can_index out of range in %s", __func__);
        return NULL;
    }
    // CAN ID 超出范围则警告
    if(config->can_tx_id < DM_CAN_ID_MIN || config->can_tx_id > DM_CAN_ID_MAX) 
        printf_log("can_tx_id out of range in %s\n", __func__);


    DM_motor *obj = (DM_motor *)RT_MALLOC(sizeof(DM_motor));
    if (!registry[config->bsp_can_index]) {
        registry[config->bsp_can_index] = 1;
        BSP_CAN_RegisterRxCallback(config->bsp_can_index, DMmotor_RxCallBack);
    }
    memset(obj, 0, sizeof(DM_motor));
    obj->config = *config;
    obj->motor_controller = create_controller(&obj->config.motor_controller_config);
    obj->control_calc = func;
    BSP_CAN_AddFilter(obj->config.bsp_can_index, obj->config.can_rx_id);
    obj->monitor = Monitor_Register(obj->config.lost_callback_, 20, obj);
    obj->check_motor_inplace = DMMotor_Check_InPlace;
    if(config->fre_rel == 0) {
        obj->config.fre_rel = 1;  // 默认相对频率为1
    }
    cvector_pushback(dm_motor_list, (void *)&obj);
    // DMmotor_Enable(obj);
    return obj;
}

void DMmotor_Calc_Send(void) {
    
    // 每0.3s发送一次使能信号,防止电机不成功启动
    static basic_data_t enable_time = 0;
    if (BSP_sys_time_ms() - enable_time > 300) {
        for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
            DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
            DMmotor_Enable(obj); 
            enable_time = BSP_sys_time_ms();
            // obj->update = 0;
            // uint8_t cnt = 0;
            // while (obj->update != 1 && cnt < 50)
            // {
            //     BSP_Delay_us(5);
            //     cnt++;
            // }
        }
    }

    for (size_t i = 0; i < dm_motor_list->cv_len; i++)
    {
        DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);

        // 频率控制
        obj->frequency_cnt++;
        if (obj->frequency_cnt >= obj->config.fre_rel) {
            obj->frequency_cnt = 0;
        } else {
            continue;
        }
        // if (obj->errcode) {
        //     DMmotor_Clear_Err(obj);
        //     continue;
        // }

		// 控制大喵电机
		//即使stop也应该更新motor_controller
		if(obj->config.arg != NULL && obj->config.pos_fdb_calc != NULL) {
                obj->motor_controller->fdb_position = (obj->config.pos_fdb_calc)(obj->config.arg);
            } else {
                obj->motor_controller->fdb_position = obj->real_fdb_position * RAD2DEG;
            }
            if(obj->config.arg != NULL && obj->config.speed_fdb_calc != NULL) {
                obj->motor_controller->fdb_speed = (obj->config.speed_fdb_calc)(obj->config.arg);
            } else {
                obj->motor_controller->fdb_speed = obj->fdb_speed * RAD2DEG;
            }

            if (obj->control_calc) {
                obj->control_calc(obj);
            } else {
                controller_calc(obj->motor_controller);
                if (obj->config.output_mode == dm_output_reverse) {
                    obj->ref_torque = (-1.0f) * obj->motor_controller->output;
                } else {
                    obj->ref_torque = obj->motor_controller->output;
                }
            }
        if (obj->enable == dm_enable)
        {

            if (obj->config.control_mode == dm_mit)
            {
                if (fabsf(obj->ref_position) > obj->config.max_position)
                    obj->ref_position = obj->config.max_position * (obj->ref_position > 0 ? 1 : -1);
                if (fabsf(obj->ref_speed) > obj->config.max_speed)
                    obj->ref_speed = obj->config.max_speed * (obj->ref_speed > 0 ? 1 : -1);
                if (fabsf(obj->ref_torque) > obj->config.max_torque)
                    obj->ref_torque = obj->config.max_torque * (obj->ref_torque > 0 ? 1 : -1);
                if (obj->kp > obj->config.max_kp)
                    obj->kp = obj->config.max_kp;
                if (obj->kd > obj->config.max_kd)
                    obj->kd = obj->config.max_kd;

                uint8_t send_data[8];
                uint16_t pos_send = float_to_uint(obj->ref_position, -obj->config.max_position, obj->config.max_position, 16);
                uint16_t speed_send = float_to_uint(obj->ref_speed, -obj->config.max_speed, obj->config.max_speed, 12);
                uint16_t kp_send = float_to_uint(obj->kp, 0, obj->config.max_kp, 12);
                uint16_t kd_send = float_to_uint(obj->kd, 0, obj->config.max_kd, 12);
                uint16_t torque_send = float_to_uint(obj->ref_torque, -obj->config.max_torque, obj->config.max_torque, 12);

                send_data[0] = (pos_send >> 8);
                send_data[1] = pos_send;
                send_data[2] = speed_send >> 4;
                send_data[3] = ((speed_send & 0xF) << 4) | (kp_send >> 8);
                send_data[4] = kp_send;
                send_data[5] = kd_send >> 4;
                send_data[6] = ((kd_send & 0xF) << 4) | (torque_send >> 8);
                send_data[7] = torque_send;
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_MIT_OFFSET, send_data, 8);
            }
            else if (obj->config.control_mode == dm_pos)
            {
                uint8_t send_data[8] = {0};
                uint8_t *ppos = (uint8_t *)&obj->ref_position;
                uint8_t *pspeed = (uint8_t *)&obj->ref_speed;
                memcpy(send_data, ppos, 4);
                memcpy(send_data + 4, pspeed, 4);
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_POS_SPEED_OFFSET, send_data, 8);
            }
            else if (obj->config.control_mode == dm_speed)
            {
                uint8_t send_data[4] = {0};
                uint8_t *pspeed = (uint8_t *)&obj->ref_speed;
                memcpy(send_data, pspeed, 4);
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_SPEED_OFFSET, send_data, 4);
            }
            // obj->update = 0;
            // uint8_t cnt = 0;
            // while (obj->update != 1 && cnt < 50)
            // {
            //     BSP_Delay_us(5);
            //     cnt++;
            // }
        }
        // 电机stop模式
        else
        {
            if (obj->config.control_mode == dm_mit)
            {
                uint8_t send_data[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_MIT_OFFSET, send_data, 8);
            }
            else if (obj->config.control_mode == dm_pos)
            {
                // uint8_t send_data[8] = {0};
                // BSP_CAN_Send(obj->config.bsp_can_index, obj->config.can_tx_id + DMID_OFFSET + 0x100, send_data, 8);
                DMmotor_Disable(obj);
            }
            else if (obj->config.control_mode == dm_speed)
            {
                // uint8_t send_data[4] = {0};
                // BSP_CAN_Send(obj->config.bsp_can_index, obj->config.can_tx_id + DMID_OFFSET + 0x200, send_data, 4);
                DMmotor_Disable(obj);
            }
            // obj->update = 0;
            // uint8_t cnt = 0;
            // while (obj->update != 1 && cnt < 50)
            // {
            //     BSP_Delay_us(5);
            //     cnt++;
            // }
        }
    }
}

#else
void DMmotor_Driver_Init(void){}
void DMmotor_Calc_Send(void){}

#endif