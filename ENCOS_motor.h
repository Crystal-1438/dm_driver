#pragma once
#include "cvector.h"
#include "bsp_can.h"
#include "bsp_time.h"
#include "monitor.h"
#include "board_def.h"
#include "controller.h"
#include "bsp_supervise.h"
#include "encos_protocol.h"
#include <stdint.h>

/* ENCOS V1.20EAP driver.
 * Formerly published under MT4 names; API renamed to ENCOS_motor / ENCOSmotor_*.
 * All ranges MUST match the motor's actual stored protocol settings.
 * No automatic motor configuration, zeroing, brake release or ID changes.
 * APIs/Rx/Calc_Send must be serialized by the caller (see README_ENCOS.md).
 */
struct encos_motor_t;
typedef void (*control_encos_func)(struct encos_motor_t *motor);
typedef float (*calc_fdb_func_encos)(void *arg);
typedef enum { encos_stop, encos_enable } ENCOS_enable;
typedef enum { encos_output_normal, encos_output_reverse } ENCOS_output_mode;
typedef enum { encos_soft_stop, encos_damping_stop } ENCOS_stop_mode;

typedef struct encos_motor_config_t {
    uint8_t bsp_can_index;
    uint16_t can_tx_id, can_rx_id; /* 1..0x7FE; must be equal */
    controller_config motor_controller_config;
    calc_fdb_func_encos pos_fdb_calc, speed_fdb_calc;
    void *arg;
    lost_callback lost_callback_;
    ENCOS_output_mode output_mode;
    uint16_t fre_rel; /* send every N Calc_Send calls, 0 means 1 */
    ENCOS_Ranges ranges; /* REQUIRED: kp/kd/position/speed/torque/current min/max */
    float torque_constant; /* Nm/A; 0 = unknown, estimated torque is invalid */
    float position_offset; /* rad; host coordinate = motor position + offset */
    uint32_t feedback_timeout_ms; /* 0 means 100ms; independent of motor CAN timeout */
    ENCOS_stop_mode stop_mode; /* default zero-current, optional variable damping brake */
} ENCOS_motor_config;

typedef struct encos_motor_t {
    ENCOS_motor_config config;
    ENCOS_enable enable;
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
    control_encos_func control_calc;
    uint8_t (*check_motor_inplace)(struct encos_motor_t *, float range);
    uint8_t zero_pending, zero_ack_received, zero_ack_success;
    uint32_t zero_sent_ms;
    ENCOS_Result last_result;
} ENCOS_motor;

#define ENCOS_CAN_ID_MIN 0x001u
#define ENCOS_CAN_ID_MAX 0x7FEu
#define ENCOS_BROADCAST_ID ENCOS_SETTINGS_ID
#define ENCOS_OFFLINE_TIMEOUT_MS 100u
#define ENCOS_SETTING_GUARD_MS 501u /* manual section 3.5 requires >500ms */
#define ENCOS_ZERO_ACK_TIMEOUT_MS 1000u /* host policy, not a protocol guarantee */

void ENCOSmotor_Driver_Init(void);
ENCOS_motor *ENCOSmotor_Create(ENCOS_motor_config *config, control_encos_func func);
void ENCOSmotor_Calc_Send(void);
/* Enable is a LOCAL state change, requires fresh position/speed and no reported error. */
ENCOS_Result ENCOSmotor_Enable(ENCOS_motor *obj);
/* Disable means software STOP, not physical power-stage disable. Sends stop frame now. */
ENCOS_Result ENCOSmotor_Disable(ENCOS_motor *obj);
/* Stop all motors on this CAN bus before zeroing. Bus TX pauses >500ms afterward. */
ENCOS_Result ENCOSmotor_Save_Zero(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Query(ENCOS_motor *obj, uint8_t decimal_code);
/* Pure builders in encos_protocol.h cover servo/current/torque/brake/ID commands.
 * Do not bypass this driver's bus setting guard when transmitting those frames. */
void ENCOSmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);
ENCOS_Result ENCOSmotor_FeedbackData_Update(ENCOS_motor *obj, const uint8_t *data, size_t len);
const char *ENCOSmotor_Err_String(uint8_t errcode);
uint8_t ENCOSmotor_Is_Online(ENCOS_motor *obj);
uint8_t ENCOSMotor_Check_InPlace(ENCOS_motor *obj, float range);
/* Legacy unsupported operations are explicit, never emit invented commands. */
ENCOS_Result ENCOSmotor_Clear_Err(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Reset_Round(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Broadcast_Cmd(ENCOS_motor *obj, uint8_t sub_cmd);
