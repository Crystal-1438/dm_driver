#include "MT4_motor.h"
#if _HAL_MT4MOTOR_ENABLE
#include "bsp_log.h"
#include "common.h"

// can_tx_id == 控制帧 ID，同时也是反馈帧 ID
// 本协议下电机的发送与接收共用一个 ID，config.can_rx_id 必须等于 config.can_tx_id
/* 控制帧ID规则为
    仅一种模式：0x000 + CAN_ID
    （不像达妙那样有 0x100 / 0x200 的模式偏移）
*/
/* 反馈帧ID为
    CAN_ID（电机自身 ID）
*/

cvector *mt4_motor_list;

/* SDK 的定标用 (1<<bits)-1 作分母，与 common.h 里的实现未必一致，
 * 为保证与 libarx_x5_src.so 逐位对齐，这里单独实现一份。*/
static uint16_t MT4_Float_To_Uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return (uint16_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

static float MT4_Uint_To_Float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + x_min;
}

uint8_t MT4Motor_Check_InPlace(MT4_motor *obj, float range) {
    if (obj->monitor->count < 1) return 0;
    return fabs(obj->ref_position - obj->fdb_position) <= range;
}

void MT4motor_FeedbackData_Update(MT4_motor *obj, uint8_t *data)
{
    obj->last_fdb_position = obj->fdb_position;
    obj->update = 0;
    obj->monitor->reset(obj->monitor);
    memcpy(obj->rx_data, data, 8);

    obj->errcode = data[0] & MT4_ERRCODE_MASK;  // 错误码在低5位，与达妙不同

    uint16_t pos_raw = ((uint16_t)data[1] << 8) | data[2];
    uint16_t spd_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t tor_raw = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];

    obj->fdb_position = MT4_Uint_To_Float(pos_raw, -obj->config.max_position, obj->config.max_position, 16);
    obj->fdb_speed    = MT4_Uint_To_Float(spd_raw, -obj->config.max_speed,    obj->config.max_speed,    12);
    obj->fdb_torque   = MT4_Uint_To_Float(tor_raw, -obj->config.max_torque,   obj->config.max_torque,   12);
    obj->t_mos        = (int8_t)(((int)data[6] - MT4_TEMP_OFFSET) / MT4_TEMP_DIV);
    // data[7] 本协议未使用

    // 多圈展开：本次与上次相差超过半个量程，即认为跨过了一圈
    if (obj->fdb_position - obj->last_fdb_position > obj->config.max_position)
        obj->round--;
    else if (obj->fdb_position - obj->last_fdb_position < -obj->config.max_position)
        obj->round++;
    obj->real_fdb_position = obj->fdb_position
                           + obj->round * 2 * obj->config.max_position
                           + obj->config.position_offset;

    // 2π 软回绕：超出 ±wrap_threshold 就折回一圈，并置标志供控制帧反向补偿
    if (obj->real_fdb_position > obj->config.wrap_threshold) {
        obj->real_fdb_position -= MT4_2PI;
        obj->wrap = 1;
    } else if (obj->real_fdb_position < -obj->config.wrap_threshold) {
        obj->real_fdb_position += MT4_2PI;
        obj->wrap = -1;
    }

    FrameRateStatistics(&obj->motor_fps);
}

void MT4motor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, basic_data_t len) {
    (void)len;
    (void)can_id;

    for (size_t i = 0; i < mt4_motor_list->cv_len; i++) {
        MT4_motor *obj = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);
        if (obj->config.can_rx_id == identifier) {
            MT4motor_FeedbackData_Update(obj, data);
        }
    }
}

/* 组帧并发送一帧控制报文。
 * 使能/失能没有专用帧，都是通过本函数发不同的 kd 实现的。*/
static void MT4motor_Send_Control(MT4_motor *obj, float kp, float kd,
                                  float pos, float spd, float tor)
{
    // 位置需要抵消掉反馈侧做过的 2π 回绕与位置偏置
    float pos_eff = pos - obj->config.position_offset
                  + MT4_2PI * (float)obj->wrap;

    uint16_t kp_send  = MT4_Float_To_Uint(kp,      0,                          obj->config.max_kp,       12);
    uint16_t kd_send  = MT4_Float_To_Uint(kd,      0,                          obj->config.max_kd,       12);
    uint16_t pos_send = MT4_Float_To_Uint(pos_eff, -obj->config.max_position,  obj->config.max_position, 16);
    uint16_t spd_send = MT4_Float_To_Uint(spd,     -obj->config.max_speed,     obj->config.max_speed,    12);
    uint16_t tor_send = MT4_Float_To_Uint(tor,     -obj->config.max_torque,    obj->config.max_torque,   12);

    /* kd 在总线上只有 9 bit。SDK 直接截断高 3 位，kd 超过约 6.24 会回绕成一个很小的值；
     * 这里改成饱和，行为在 SDK 实际用到的量程内（kd <= 5）完全一致，且不会突然失去阻尼。*/
    if (kd_send > ((1 << MT4_KD_WIRE_BITS) - 1))
        kd_send = (1 << MT4_KD_WIRE_BITS) - 1;

    uint8_t send_data[8];
    send_data[0] = (kp_send >> 7) & 0xFF;
    send_data[1] = ((kp_send << 1) & 0xFE) | ((kd_send >> 8) & 0x01);
    send_data[2] = kd_send & 0xFF;
    send_data[3] = pos_send >> 8;
    send_data[4] = pos_send;
    send_data[5] = spd_send >> 4;
    send_data[6] = ((spd_send & 0x0F) << 4) | (tor_send >> 8);
    send_data[7] = tor_send;

    BSP_CAN_Send(obj->config.bsp_can_index, obj->config.can_tx_id, send_data, 8);
}

void MT4motor_Enable(MT4_motor *obj)
{
    // 无专用使能帧，发一帧轻阻尼控制帧即可，此时可手动拖动
    MT4motor_Send_Control(obj, 0.0f, MT4_ENABLE_KD, 0.0f, 0.0f, 0.0f);
}

void MT4motor_Disable(MT4_motor *obj)
{
    // 无专用失能帧，发最大阻尼帧刹车。注意**电机仍然带电**，不是断电
    MT4motor_Send_Control(obj, 0.0f, MT4_DISABLE_KD, 0.0f, 0.0f, 0.0f);
}

// TODO: 测试MT4motor_Save_Zero函数
void MT4motor_Save_Zero(MT4_motor *obj)
{
    // 走 0x7FF 广播帧，DLC=4，与控制帧完全是两套格式
    uint8_t data[4] = {0x00, obj->config.can_tx_id, 0x00, 0x03};
    BSP_CAN_Send(obj->config.bsp_can_index, MT4_BROADCAST_ID, data, 4);
}

void MT4motor_Reset_Round(MT4_motor *obj)
{
    // 与 SDK 的 resetCircle() 对齐：只清圈数，**不清 wrap 标志**
    obj->round = 0;
}

void MT4motor_Driver_Init(void)
{
    mt4_motor_list = cvector_create(sizeof(MT4_motor *));
}

MT4_motor* MT4motor_Create(MT4_motor_config* config, control_mt4_func func)
{
    static uint8_t registry[DEVICE_CAN_CNT];
    if (config->bsp_can_index >= DEVICE_CAN_CNT) {// CAN索引超出范围
        printf_log("bsp_can_index out of range in %s", __func__);
        return NULL;
    }
    if (config->can_tx_id < MT4_CAN_ID_MIN || config->can_tx_id > MT4_CAN_ID_MAX)
        printf_log("can_tx_id out of range in %s\n", __func__);
    // 本协议反馈帧 ID 等于电机自身 ID，两者不一致基本是配置写错了
    if (config->can_rx_id != config->can_tx_id)
        printf_log("can_rx_id should equal can_tx_id in %s\n", __func__);

    MT4_motor *obj = (MT4_motor *)RT_MALLOC(sizeof(MT4_motor));
    if (!registry[config->bsp_can_index]) {
        registry[config->bsp_can_index] = 1;
        BSP_CAN_RegisterRxCallback(config->bsp_can_index, MT4motor_RxCallBack);
    }
    memset(obj, 0, sizeof(MT4_motor));
    obj->config = *config;

    // 未填的量程按 SDK 逆向出的默认值补齐
    if (obj->config.max_position    == 0) obj->config.max_position    = MT4_DEFAULT_MAX_POSITION;
    if (obj->config.max_speed       == 0) obj->config.max_speed       = MT4_DEFAULT_MAX_SPEED;
    if (obj->config.max_torque      == 0) obj->config.max_torque      = MT4_DEFAULT_MAX_TORQUE;
    if (obj->config.max_kp          == 0) obj->config.max_kp          = MT4_DEFAULT_MAX_KP;
    if (obj->config.max_kd          == 0) obj->config.max_kd          = MT4_DEFAULT_MAX_KD;
    if (obj->config.wrap_threshold  == 0) obj->config.wrap_threshold  = MT4_DEFAULT_WRAP_THRESHOLD;

    obj->motor_controller = create_controller(&obj->config.motor_controller_config);
    obj->control_calc = func;
    BSP_CAN_AddFilter(obj->config.bsp_can_index, obj->config.can_rx_id);
    obj->monitor = Monitor_Register(obj->config.lost_callback_, MT4_OFFLINE_TIMEOUT_MS, obj);
    obj->check_motor_inplace = MT4Motor_Check_InPlace;
    if (config->fre_rel == 0) {
        obj->config.fre_rel = 1;  // 默认相对频率为1
    }
    cvector_pushback(mt4_motor_list, (void *)&obj);
    return obj;
}

void MT4motor_Calc_Send(void) {

    // 每0.3s补发一次使能帧,防止电机不成功启动
    static basic_data_t enable_time = 0;
    if (BSP_sys_time_ms() - enable_time > 300) {
        for (size_t i = 0; i < mt4_motor_list->cv_len; i++) {
            MT4_motor *obj = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);
            MT4motor_Enable(obj);
            enable_time = BSP_sys_time_ms();
        }
    }

    for (size_t i = 0; i < mt4_motor_list->cv_len; i++)
    {
        MT4_motor *obj = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);

        // 频率控制
        obj->frequency_cnt++;
        if (obj->frequency_cnt >= obj->config.fre_rel) {
            obj->frequency_cnt = 0;
        } else {
            continue;
        }

        //即使stop也应该更新motor_controller
        if (obj->config.arg != NULL && obj->config.pos_fdb_calc != NULL) {
            obj->motor_controller->fdb_position = (obj->config.pos_fdb_calc)(obj->config.arg);
        } else {
            obj->motor_controller->fdb_position = obj->real_fdb_position * RAD2DEG;
        }
        if (obj->config.arg != NULL && obj->config.speed_fdb_calc != NULL) {
            obj->motor_controller->fdb_speed = (obj->config.speed_fdb_calc)(obj->config.arg);
        } else {
            obj->motor_controller->fdb_speed = obj->fdb_speed * RAD2DEG;
        }

        if (obj->control_calc) {
            obj->control_calc(obj);
        } else {
            controller_calc(obj->motor_controller);
            if (obj->config.output_mode == mt4_output_reverse) {
                obj->ref_torque = (-1.0f) * obj->motor_controller->output;
            } else {
                obj->ref_torque = obj->motor_controller->output;
            }
        }

        if (obj->enable == mt4_enable)
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

            MT4motor_Send_Control(obj, obj->kp, obj->kd,
                                  obj->ref_position, obj->ref_speed, obj->ref_torque);
        }
        // 电机stop模式
        else
        {
            MT4motor_Disable(obj);
        }
    }
}

#else
void MT4motor_Driver_Init(void){}
void MT4motor_Calc_Send(void){}

#endif
