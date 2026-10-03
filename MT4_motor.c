#include "MT4_motor.h"
#if _HAL_MT4MOTOR_ENABLE
#include "bsp_log.h"
#include <math.h>
#include <string.h>

cvector *mt4_motor_list;
static uint8_t registry[DEVICE_CAN_CNT];
static uint8_t bus_guard[DEVICE_CAN_CNT];
static uint32_t bus_setting_ms[DEVICE_CAN_CNT];

static uint32_t now_ms(void) { return (uint32_t)BSP_sys_time_ms(); }
static int bus_busy(uint8_t bus) {
    if (!bus_guard[bus]) return 0;
    if ((uint32_t)(now_ms() - bus_setting_ms[bus]) < MT4_SETTING_GUARD_MS) return 1;
    bus_guard[bus] = 0;
    return 0;
}
static ENCOS_Result send_frame(MT4_motor *obj, const ENCOS_Frame *frame) {
    if (bus_busy(obj->config.bsp_can_index)) return ENCOS_BUSY;
    BSP_CAN_Send(obj->config.bsp_can_index, frame->id, (uint8_t *)frame->data, frame->len);
    return ENCOS_OK; /* submitted to BSP; NOT an acknowledgement from motor */
}
static void reset_controller(MT4_motor *obj) {
    controller *c = obj->motor_controller;
    PID_Init(&c->pid_pos_data, &obj->config.motor_controller_config.position_pid_config);
    PID_Init(&c->pid_speed_data, &obj->config.motor_controller_config.speed_pid_config);
    PID_Config p = c->pid_pos_data.config, s = c->pid_speed_data.config;
    memset(&c->pid_pos_data, 0, sizeof(c->pid_pos_data));
    memset(&c->pid_speed_data, 0, sizeof(c->pid_speed_data));
    c->pid_pos_data.config = p; c->pid_speed_data.config = s;
    c->output = 0;
}
static ENCOS_Result send_stop(MT4_motor *obj) {
    ENCOS_Frame frame;
    ENCOS_CurrentMode mode = obj->config.stop_mode == mt4_damping_stop ?
                            ENCOS_DAMPING_BRAKE : ENCOS_CURRENT;
    ENCOS_Result result = ENCOS_PackCurrent(&frame, obj->config.can_tx_id, mode, 0, 1);
    return result == ENCOS_OK ? send_frame(obj, &frame) : result;
}
static int fresh(MT4_motor *obj, uint8_t field, uint32_t at) {
    return (obj->feedback_valid & field) &&
           (uint32_t)(now_ms() - at) < obj->config.feedback_timeout_ms;
}
static int motion_ready(MT4_motor *obj) {
    return !obj->errcode && !obj->zero_pending &&
           fresh(obj, ENCOS_HAS_POSITION, obj->last_position_ms) &&
           fresh(obj, ENCOS_HAS_SPEED, obj->last_speed_ms);
}
uint8_t MT4motor_Is_Online(MT4_motor *obj) {
    return obj && obj->seen_reply &&
           (uint32_t)(now_ms() - obj->last_rx_ms) < obj->config.feedback_timeout_ms;
}
uint8_t MT4Motor_Check_InPlace(MT4_motor *obj, float range) {
    return obj && isfinite(range) && range >= 0 && !obj->errcode &&
           fresh(obj, ENCOS_HAS_POSITION, obj->last_position_ms) &&
           fabsf(obj->ref_position - obj->real_fdb_position) <= range;
}
ENCOS_Result MT4motor_FeedbackData_Update(MT4_motor *obj, const uint8_t *data, size_t len) {
    if (!obj) return ENCOS_INVALID;
    ENCOS_Feedback f;
    ENCOS_Result result = ENCOS_Decode(data, len, &obj->config.ranges, &f);
    if (result != ENCOS_OK) return result;
    obj->feedback = f;
    memset(obj->rx_data, 0, sizeof(obj->rx_data));
    memcpy(obj->rx_data, data, len); obj->rx_len = (uint8_t)len;
    obj->errcode = f.error;
    obj->last_rx_ms = now_ms(); obj->seen_reply = 1; obj->update = 1;
    obj->monitor->reset(obj->monitor);
    if (f.valid & ENCOS_HAS_POSITION) {
        obj->last_fdb_position = obj->fdb_position;
        obj->fdb_position = f.position;
        obj->real_fdb_position = f.position + obj->config.position_offset;
        obj->last_position_ms = obj->last_rx_ms;
    }
    if (f.valid & ENCOS_HAS_SPEED) {
        obj->fdb_speed = f.speed; obj->last_speed_ms = obj->last_rx_ms;
    }
    if (f.valid & ENCOS_HAS_CURRENT) {
        obj->fdb_current = f.current;
        obj->torque_valid = obj->config.torque_constant > 0;
        obj->fdb_torque = obj->torque_valid ? f.current * obj->config.torque_constant : NAN;
    }
    if (f.valid & ENCOS_HAS_MOTOR_TEMP) obj->t_rotor = f.motor_temp;
    if (f.valid & ENCOS_HAS_MOS_TEMP) obj->t_mos = f.mos_temp;
    obj->feedback_valid |= f.valid;
    FrameRateStatistics(&obj->motor_fps);
    return ENCOS_OK;
}
void MT4motor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, basic_data_t len) {
    if (!mt4_motor_list || !data || len < 2 || len > 8) return;
    for (size_t i = 0; i < mt4_motor_list->cv_len; ++i) {
        MT4_motor *obj = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);
        if (obj->config.bsp_can_index != can_id) continue;
        if (identifier == obj->config.can_rx_id) {
            (void)MT4motor_FeedbackData_Update(obj, data, (size_t)len);
        } else if (identifier == ENCOS_SETTINGS_ID && len == 4 && data[2] == 1 &&
                   (((uint16_t)data[0] << 8) | data[1]) == obj->config.can_tx_id &&
                   obj->zero_pending && (data[3] == 3 || data[3] == 0)) {
            obj->zero_ack_received = 1; obj->zero_ack_success = data[3] == 3;
            obj->zero_pending = 0;
            /* Prior coordinates are stale after zeroing; require new telemetry. */
            obj->feedback_valid &= (uint8_t)~(ENCOS_HAS_POSITION | ENCOS_HAS_SPEED);
            obj->last_rx_ms = now_ms(); obj->seen_reply = 1;
            obj->monitor->reset(obj->monitor);
            obj->update = 1;
        }
    }
}
void MT4motor_Driver_Init(void) {
    if (!mt4_motor_list) mt4_motor_list = cvector_create(sizeof(MT4_motor *));
}
MT4_motor *MT4motor_Create(MT4_motor_config *config, control_mt4_func func) {
    if (!config || config->bsp_can_index >= DEVICE_CAN_CNT ||
        !ENCOS_ValidId(config->can_tx_id) || config->can_rx_id != config->can_tx_id ||
        !ENCOS_ValidRanges(&config->ranges) || !isfinite(config->position_offset) ||
        !isfinite(config->torque_constant) || config->torque_constant < 0 ||
        config->feedback_timeout_ms > 0x7FFFFFFFu ||
        (config->stop_mode != mt4_soft_stop && config->stop_mode != mt4_damping_stop) ||
        (config->output_mode != mt4_output_normal && config->output_mode != mt4_output_reverse)) {
        printf_log("Invalid ENCOS/MT4 config\n"); return NULL;
    }
    MT4motor_Driver_Init();
    if (!mt4_motor_list) return NULL;
    for (size_t i = 0; i < mt4_motor_list->cv_len; ++i) {
        MT4_motor *old = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);
        if (old->config.bsp_can_index == config->bsp_can_index &&
            old->config.can_rx_id == config->can_rx_id) return NULL;
    }
    MT4_motor *obj = RT_MALLOC(sizeof(*obj));
    if (!obj) return NULL;
    memset(obj, 0, sizeof(*obj));
    obj->config = *config;
    if (!obj->config.fre_rel) obj->config.fre_rel = 1;
    if (!obj->config.feedback_timeout_ms) obj->config.feedback_timeout_ms = MT4_OFFLINE_TIMEOUT_MS;
    /* Allocation failures are checked before publishing the motor to callbacks. */
    obj->motor_controller = create_controller(&obj->config.motor_controller_config);
    if (!obj->motor_controller) { RT_FREE(obj); return NULL; }
    obj->monitor = Monitor_Register(obj->config.lost_callback_,
                                    (int)obj->config.feedback_timeout_ms, obj);
    if (!obj->monitor) { RT_FREE(obj->motor_controller); RT_FREE(obj); return NULL; }
    obj->control_calc = func; obj->check_motor_inplace = MT4Motor_Check_InPlace;
    obj->fdb_torque = NAN; obj->t_rotor = NAN; obj->t_mos = NAN;
    cvector_pushback(mt4_motor_list, &obj);
    if (!registry[config->bsp_can_index]) {
        BSP_CAN_RegisterRxCallback(config->bsp_can_index, MT4motor_RxCallBack);
        BSP_CAN_AddFilter(config->bsp_can_index, ENCOS_SETTINGS_ID);
        registry[config->bsp_can_index] = 1;
    }
    BSP_CAN_AddFilter(config->bsp_can_index, config->can_rx_id);
    return obj;
}
ENCOS_Result MT4motor_Enable(MT4_motor *obj) {
    if (!obj) return ENCOS_INVALID;
    if (bus_busy(obj->config.bsp_can_index) || !motion_ready(obj)) return ENCOS_BUSY;
    reset_controller(obj); obj->enable = mt4_enable;
    return ENCOS_OK; /* next Calc_Send sends the actual control command */
}
ENCOS_Result MT4motor_Disable(MT4_motor *obj) {
    if (!obj) return ENCOS_INVALID;
    obj->enable = mt4_stop; reset_controller(obj);
    return obj->last_result = send_stop(obj);
}
ENCOS_Result MT4motor_Save_Zero(MT4_motor *obj) {
    if (!obj) return ENCOS_INVALID;
    if (bus_busy(obj->config.bsp_can_index)) return ENCOS_BUSY;
    for (size_t i = 0; i < mt4_motor_list->cv_len; ++i) {
        MT4_motor *other = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);
        if (other->config.bsp_can_index == obj->config.bsp_can_index && other->enable != mt4_stop)
            return ENCOS_BUSY;
    }
    ENCOS_Frame frame;
    ENCOS_Result r = ENCOS_PackZero(&frame, obj->config.can_tx_id);
    if (r != ENCOS_OK) return r;
    obj->zero_pending = 1; obj->zero_ack_received = 0; obj->zero_ack_success = 0;
    obj->zero_sent_ms = now_ms();
    obj->feedback_valid &= (uint8_t)~(ENCOS_HAS_POSITION | ENCOS_HAS_SPEED);
    r = send_frame(obj, &frame);
    bus_setting_ms[obj->config.bsp_can_index] = now_ms();
    bus_guard[obj->config.bsp_can_index] = 1;
    return obj->last_result = r;
}
ENCOS_Result MT4motor_Query(MT4_motor *obj, uint8_t code) {
    if (!obj) return ENCOS_INVALID;
    ENCOS_Frame frame;
    ENCOS_Result r = ENCOS_PackQuery(&frame, obj->config.can_tx_id, code);
    return obj->last_result = r == ENCOS_OK ? send_frame(obj, &frame) : r;
}
const char *MT4motor_Err_String(uint8_t code) { return ENCOS_ErrorString(code); }
ENCOS_Result MT4motor_Clear_Err(MT4_motor *obj) {
    (void)obj; return ENCOS_UNSUPPORTED; /* No clear-fault command specified in manual. */
}
ENCOS_Result MT4motor_Reset_Round(MT4_motor *obj) {
    (void)obj; return ENCOS_UNSUPPORTED; /* No invented 25rad / 2pi position wrapping. */
}
ENCOS_Result MT4motor_Broadcast_Cmd(MT4_motor *obj, uint8_t sub_cmd) {
    return sub_cmd == 3 ? MT4motor_Save_Zero(obj) : ENCOS_UNSUPPORTED;
}
void MT4motor_Calc_Send(void) {
    if (!mt4_motor_list) return;
    for (size_t i = 0; i < mt4_motor_list->cv_len; ++i) {
        MT4_motor *obj = *(MT4_motor **)cvector_val_at(mt4_motor_list, i);
        if (bus_busy(obj->config.bsp_can_index)) continue;
        /* Missing zero ACK is observable; do not retain an eternal pending state. */
        if (obj->zero_pending && (uint32_t)(now_ms() - obj->zero_sent_ms) >= MT4_ZERO_ACK_TIMEOUT_MS)
            obj->zero_pending = 0;
        if (++obj->frequency_cnt < obj->config.fre_rel) continue;
        obj->frequency_cnt = 0;
        if (obj->enable != mt4_enable || !motion_ready(obj)) {
            obj->enable = mt4_stop; reset_controller(obj);
            obj->last_result = send_stop(obj); continue;
        }
        controller *c = obj->motor_controller;
        c->fdb_position = obj->config.pos_fdb_calc ?
            obj->config.pos_fdb_calc(obj->config.arg) : obj->real_fdb_position * (180 / ENCOS_PI);
        c->fdb_speed = obj->config.speed_fdb_calc ?
            obj->config.speed_fdb_calc(obj->config.arg) : obj->fdb_speed * (180 / ENCOS_PI);
        if (!isfinite(c->fdb_position) || !isfinite(c->fdb_speed)) {
            (void)MT4motor_Disable(obj); obj->last_result = ENCOS_INVALID; continue;
        }
        if (obj->control_calc) obj->control_calc(obj);
        else {
            controller_calc(c);
            obj->ref_torque = obj->config.output_mode == mt4_output_reverse ? -c->output : c->output;
        }
        if (bus_busy(obj->config.bsp_can_index)) continue;
        if (obj->enable != mt4_enable) { obj->last_result = send_stop(obj); continue; }
        ENCOS_Frame frame;
        ENCOS_Result r = ENCOS_PackMixed(&frame, obj->config.can_tx_id, &obj->config.ranges,
            obj->kp, obj->kd, obj->ref_position - obj->config.position_offset,
            obj->ref_speed, obj->ref_torque);
        if (r != ENCOS_OK) {
            (void)MT4motor_Disable(obj); obj->last_result = r;
        } else obj->last_result = send_frame(obj, &frame);
    }
}
#else
void MT4motor_Driver_Init(void) {}
void MT4motor_Calc_Send(void) {}
#endif
