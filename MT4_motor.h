#pragma once
#include "cvector.h"
#include "bsp_can.h"
#include "bsp_time.h"
#include "monitor.h"
#include "board_def.h"
#include "controller.h"
#include "bsp_supervise.h"
#include "encos_protocol.h"

/* ENCOS V1.20EAP driver. MT4 names retained for existing call sites.
 * All ranges MUST match the motor's actual stored protocol settings.
 * No automatic motor configuration, zeroing, brake release or ID changes.
 * APIs/Rx/Calc_Send must be serialized by the caller (see README_ENCOS.md).
 */
struct mt4_motor_t;
typedef void (*control_mt4_func)(struct mt4_motor_t *motor);
typedef float (*calc_fdb_func_mt4)(void *arg);
typedef enum { mt4_stop, mt4_enable } MT4_enable;
typedef enum { mt4_output_normal, mt4_output_reverse } MT4_output_mode;
typedef enum { mt4_soft_stop, mt4_damping_stop } MT4_stop_mode;

typedef struct mt4_motor_config_t {
    uint8_t bsp_can_index;
    uint16_t can_tx_id, can_rx_id; /* 1..0x7FE; must be equal */
    controller_config motor_controller_config;
    calc_fdb_func_mt4 pos_fdb_calc, speed_fdb_calc;
    void *arg;
    lost_callback lost_callback_;
    MT4_output_mode output_mode;
    uint16_t fre_rel; /* send every N Calc_Send calls, 0 means 1 */
    ENCOS_Ranges ranges; /* REQUIRED: kp/kd/position/speed/torque/current min/max */
    float torque_constant; /* Nm/A; 0 = unknown, estimated torque is invalid */
    float position_offset; /* rad; host coordinate = motor position + offset */
    uint32_t feedback_timeout_ms; /* 0 means 100ms; independent of motor CAN timeout */
    MT4_stop_mode stop_mode; /* default zero-current, optional variable damping brake */
} MT4_motor_config;

typedef struct mt4_motor_t {
    MT4_motor_config config;
    MT4_enable enable;
    uint8_t update; /* set to 1 after accepted reply; caller may clear */
    uint8_t rx_data[8], rx_len, errcode;
    ENCOS_Feedback feedback; /* latest typed reply including raw query payload */
    float last_fdb_position, fdb_position, real_fdb_position; /* rad */
    float fdb_speed, fdb_current; /* rad/s, phase A */
    float fdb_torque; /* estimated current * configured Kt; NAN if unknown */
    float t_rotor, t_mos; /* deg C, half-degree resolution */
    uint8_t torque_valid, feedback_valid, seen_reply;
    uint32_t last_rx_ms, last_position_ms, last_speed_ms;
    float ref_position, ref_speed, ref_torque, kp, kd;
    uint16_t frequency_cnt;
    monitor_item *monitor;
    FPS_t motor_fps;
    controller *motor_controller;
    control_mt4_func control_calc;
    uint8_t (*check_motor_inplace)(struct mt4_motor_t *, float range);
    uint8_t zero_pending, zero_ack_received, zero_ack_success;
    uint32_t zero_sent_ms;
    ENCOS_Result last_result;
} MT4_motor;

#define MT4_CAN_ID_MIN 0x001u
#define MT4_CAN_ID_MAX 0x7FEu
#define MT4_BROADCAST_ID ENCOS_SETTINGS_ID
#define MT4_OFFLINE_TIMEOUT_MS 100u
#define MT4_SETTING_GUARD_MS 501u /* manual section 3.5 requires >500ms */
#define MT4_ZERO_ACK_TIMEOUT_MS 1000u /* host policy, not a protocol guarantee */

void MT4motor_Driver_Init(void);
MT4_motor *MT4motor_Create(MT4_motor_config *config, control_mt4_func func);
void MT4motor_Calc_Send(void);
/* Enable is a LOCAL state change, requires fresh position/speed and no reported error. */
ENCOS_Result MT4motor_Enable(MT4_motor *obj);
/* Disable means software STOP, not physical power-stage disable. Sends stop frame now. */
ENCOS_Result MT4motor_Disable(MT4_motor *obj);
/* Stop all motors on this CAN bus before zeroing. Bus TX pauses >500ms afterward. */
ENCOS_Result MT4motor_Save_Zero(MT4_motor *obj);
ENCOS_Result MT4motor_Query(MT4_motor *obj, uint8_t decimal_code);
/* Pure builders in encos_protocol.h cover servo/current/torque/brake/ID commands.
 * Do not bypass this driver's bus setting guard when transmitting those frames. */
void MT4motor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, basic_data_t len);
ENCOS_Result MT4motor_FeedbackData_Update(MT4_motor *obj, const uint8_t *data, size_t len);
const char *MT4motor_Err_String(uint8_t errcode);
uint8_t MT4motor_Is_Online(MT4_motor *obj);
uint8_t MT4Motor_Check_InPlace(MT4_motor *obj, float range);
/* Legacy unsupported operations are explicit, never emit invented commands. */
ENCOS_Result MT4motor_Clear_Err(MT4_motor *obj);
ENCOS_Result MT4motor_Reset_Round(MT4_motor *obj);
ENCOS_Result MT4motor_Broadcast_Cmd(MT4_motor *obj, uint8_t sub_cmd);
