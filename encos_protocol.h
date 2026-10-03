#ifndef ENCOS_PROTOCOL_H
#define ENCOS_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* ENCOS V1.20EAP, classic CAN, standard identifiers, big-endian data.
 * This codec has no HAL dependency and never transmits anything. */
#define ENCOS_SETTINGS_ID 0x7FFu
#define ENCOS_PI 3.14159265358979323846f

typedef enum {
    ENCOS_OK = 0, ENCOS_INVALID = -1, ENCOS_UNSUPPORTED = -2,
    ENCOS_BUSY = -3
} ENCOS_Result;

typedef struct { float min, max; } ENCOS_Range;
typedef struct {
    ENCOS_Range kp, kd, position, speed, torque, current;
} ENCOS_Ranges;

typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
} ENCOS_Frame;

typedef enum {
    ENCOS_CURRENT = 0, ENCOS_TORQUE = 1, ENCOS_DAMPING_BRAKE = 2,
    ENCOS_ENERGY_BRAKE = 3, ENCOS_REGENERATIVE_BRAKE = 4,
    ENCOS_MECHANICAL_BRAKE = 5
} ENCOS_CurrentMode;

enum {
    ENCOS_HAS_POSITION = 1u, ENCOS_HAS_SPEED = 2u, ENCOS_HAS_CURRENT = 4u,
    ENCOS_HAS_MOTOR_TEMP = 8u, ENCOS_HAS_MOS_TEMP = 16u,
    ENCOS_HAS_BRAKE = 32u
};

typedef struct {
    uint8_t type, error, valid;
    float position; /* rad: type 2/query 1 converted from degrees */
    float speed;    /* rad/s: type 3/query 2 converted from RPM */
    float current; /* A, NOT torque */
    float motor_temp, mos_temp; /* deg C, preserves 0.5 degree resolution */
    uint8_t code, status, brake_released;
    ENCOS_Range query_range; /* type 5/code 23..28, decoded physical range */
    float query_value; /* type 5/code 1..5, 22, 31; wire's physical units */
    uint8_t payload[6], payload_len; /* query/config details; never auto-applied */
} ENCOS_Feedback;

int ENCOS_ValidId(uint16_t id);
int ENCOS_ValidRanges(const ENCOS_Ranges *r);
ENCOS_Result ENCOS_PackMixed(ENCOS_Frame *f, uint16_t id, const ENCOS_Ranges *r,
                            float kp, float kd, float pos, float speed, float torque);
/* Position: degrees, speed limit: RPM, current limit: A. */
ENCOS_Result ENCOS_PackPosition(ENCOS_Frame *f, uint16_t id, float degrees,
                               float rpm_limit, float current_limit, uint8_t ack);
ENCOS_Result ENCOS_PackSpeed(ENCOS_Frame *f, uint16_t id, float rpm,
                            float current_limit, uint8_t ack);
/* value is A, Nm, brake-current A, or release=0/1 depending on mode. */
ENCOS_Result ENCOS_PackCurrent(ENCOS_Frame *f, uint16_t id, ENCOS_CurrentMode mode,
                              float value, uint8_t ack);
/* Ordinary queries: decimal codes 1..5, 22..35, 37..39. Code 36 needs a group. */
ENCOS_Result ENCOS_PackQuery(ENCOS_Frame *f, uint16_t id, uint8_t code);
ENCOS_Result ENCOS_PackZero(ENCOS_Frame *f, uint16_t id);
ENCOS_Result ENCOS_PackSetId(ENCOS_Frame *f, uint16_t old_id, uint16_t new_id);
ENCOS_Result ENCOS_PackQueryId(ENCOS_Frame *f);
ENCOS_Result ENCOS_Decode(const uint8_t *data, size_t len,
                         const ENCOS_Ranges *ranges, ENCOS_Feedback *out);
const char *ENCOS_ErrorString(uint8_t error);

#endif
