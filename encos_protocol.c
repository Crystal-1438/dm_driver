#include "encos_protocol.h"
#include <math.h>
#include <string.h>
#include <float.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "ENCOS requires IEEE-754 binary32 float");

static uint16_t be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static int32_t signed16(const uint8_t *p) {
    uint16_t u = be16(p);
    return u < 0x8000u ? (int32_t)u : (int32_t)u - 65536;
}
static void put16(uint8_t *p, uint16_t u) {
    p[0] = (uint8_t)(u >> 8); p[1] = (uint8_t)u;
}
static uint32_t float_bits(float x) {
    uint32_t u; memcpy(&u, &x, sizeof(u)); return u;
}
static float read_float(const uint8_t *p) {
    uint32_t u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] << 8) | p[3];
    float x; memcpy(&x, &u, sizeof(x)); return x;
}
static int valid_range(ENCOS_Range r) {
    return isfinite(r.min) && isfinite(r.max) && r.min < r.max &&
           isfinite(r.max - r.min);
}
int ENCOS_ValidId(uint16_t id) { return id >= 1 && id < ENCOS_SETTINGS_ID; }
int ENCOS_ValidRanges(const ENCOS_Ranges *r) {
    return r && valid_range(r->kp) && valid_range(r->kd) &&
        valid_range(r->position) && valid_range(r->speed) &&
        valid_range(r->torque) && valid_range(r->current) &&
        r->kp.min == 0 && r->kd.min == 0 &&
        r->position.min <= 0 && r->position.max >= 0 &&
        r->speed.min <= 0 && r->speed.max >= 0 &&
        r->torque.min <= 0 && r->torque.max >= 0 &&
        r->current.min <= 0 && r->current.max >= 0;
}
static uint32_t quantize(float x, ENCOS_Range r, uint32_t top) {
    if (x <= r.min) return 0;
    if (x >= r.max) return top;
    return (uint32_t)(((double)x - r.min) * top / ((double)r.max - r.min));
}
static float decode_value(uint32_t u, ENCOS_Range r, uint32_t top) {
    return (float)((double)u * ((double)r.max - r.min) / top + r.min);
}
static ENCOS_Result init(ENCOS_Frame *f, uint16_t id, uint8_t len) {
    if (!f || !ENCOS_ValidId(id)) return ENCOS_INVALID;
    memset(f, 0, sizeof(*f)); f->id = id; f->len = len;
    return ENCOS_OK;
}
ENCOS_Result ENCOS_PackMixed(ENCOS_Frame *f, uint16_t id, const ENCOS_Ranges *r,
                            float kp, float kd, float pos, float speed, float torque) {
    if (!ENCOS_ValidRanges(r) || !isfinite(kp) || !isfinite(kd) ||
        !isfinite(pos) || !isfinite(speed) || !isfinite(torque)) return ENCOS_INVALID;
    if (init(f, id, 8) != ENCOS_OK) return ENCOS_INVALID;
    uint32_t p = quantize(pos, r->position, 65535);
    uint32_t v = quantize(speed, r->speed, 4095);
    uint32_t t = quantize(torque, r->torque, 4095);
    uint32_t k = quantize(kp, r->kp, 4095);
    uint32_t d = quantize(kd, r->kd, 511); /* KD is 9 bits, not clipped 12 bits. */
    f->data[0] = (uint8_t)(k >> 7); /* mode 0 in high three bits */
    f->data[1] = (uint8_t)((k << 1) | (d >> 8));
    f->data[2] = (uint8_t)d;
    put16(f->data + 3, (uint16_t)p);
    f->data[5] = (uint8_t)(v >> 4);
    f->data[6] = (uint8_t)((v << 4) | (t >> 8));
    f->data[7] = (uint8_t)t;
    return ENCOS_OK;
}
ENCOS_Result ENCOS_PackPosition(ENCOS_Frame *f, uint16_t id, float degrees,
                               float rpm_limit, float current_limit, uint8_t ack) {
    if (!isfinite(degrees) || !isfinite(rpm_limit) || !isfinite(current_limit) ||
        rpm_limit < 0 || rpm_limit > 3276.7f || current_limit < 0 ||
        current_limit > 409.5f || ack > 3) return ENCOS_INVALID;
    if (init(f, id, 8) != ENCOS_OK) return ENCOS_INVALID;
    uint32_t p = float_bits(degrees);
    uint32_t v = (uint32_t)lroundf(rpm_limit * 10);
    uint32_t c = (uint32_t)lroundf(current_limit * 10);
    f->data[0] = (uint8_t)(0x20 | (p >> 27));
    f->data[1] = (uint8_t)(p >> 19);
    f->data[2] = (uint8_t)(p >> 11);
    f->data[3] = (uint8_t)(p >> 3);
    f->data[4] = (uint8_t)((p << 5) | (v >> 10));
    f->data[5] = (uint8_t)(v >> 2);
    f->data[6] = (uint8_t)((v << 6) | (c >> 6));
    f->data[7] = (uint8_t)((c << 2) | ack);
    return ENCOS_OK;
}
ENCOS_Result ENCOS_PackSpeed(ENCOS_Frame *f, uint16_t id, float rpm,
                            float current_limit, uint8_t ack) {
    if (!isfinite(rpm) || !isfinite(current_limit) || current_limit < 0 ||
        current_limit > 6553.5f || ack > 3) return ENCOS_INVALID;
    if (init(f, id, 7) != ENCOS_OK) return ENCOS_INVALID;
    uint32_t u = float_bits(rpm);
    f->data[0] = (uint8_t)(0x40 | ack);
    f->data[1] = (uint8_t)(u >> 24); f->data[2] = (uint8_t)(u >> 16);
    f->data[3] = (uint8_t)(u >> 8); f->data[4] = (uint8_t)u;
    put16(f->data + 5, (uint16_t)lroundf(current_limit * 10));
    return ENCOS_OK;
}
ENCOS_Result ENCOS_PackCurrent(ENCOS_Frame *f, uint16_t id, ENCOS_CurrentMode mode,
                              float value, uint8_t ack) {
    if (mode < ENCOS_CURRENT || mode > ENCOS_MECHANICAL_BRAKE || ack > 3 ||
        !isfinite(value)) return ENCOS_INVALID;
    if ((mode == ENCOS_ENERGY_BRAKE || mode == ENCOS_REGENERATIVE_BRAKE) &&
        value < 0) return ENCOS_INVALID;
    int32_t raw = 0;
    if (mode == ENCOS_MECHANICAL_BRAKE) {
        if (value != 0 && value != 1) return ENCOS_INVALID;
        raw = (int32_t)value;
    } else if (mode != ENCOS_DAMPING_BRAKE) {
        if (value < -327.68f || value > 327.67f) return ENCOS_INVALID;
        raw = (int32_t)lroundf(value * 100);
    }
    if (init(f, id, 3) != ENCOS_OK) return ENCOS_INVALID;
    f->data[0] = (uint8_t)(0x60 | ((unsigned)mode << 2) | ack);
    put16(f->data + 1, (uint16_t)raw);
    return ENCOS_OK;
}
ENCOS_Result ENCOS_PackQuery(ENCOS_Frame *f, uint16_t id, uint8_t code) {
    if (!((code >= 1 && code <= 5) || (code >= 22 && code <= 35) ||
          (code >= 37 && code <= 39))) return ENCOS_UNSUPPORTED;
    if (init(f, id, 2) != ENCOS_OK) return ENCOS_INVALID;
    f->data[0] = 0xE0; f->data[1] = code; return ENCOS_OK;
}
ENCOS_Result ENCOS_PackZero(ENCOS_Frame *f, uint16_t id) {
    if (init(f, id, 4) != ENCOS_OK) return ENCOS_INVALID;
    f->id = ENCOS_SETTINGS_ID;
    put16(f->data, id); f->data[3] = 3; return ENCOS_OK;
}
ENCOS_Result ENCOS_PackSetId(ENCOS_Frame *f, uint16_t old_id, uint16_t new_id) {
    if (!ENCOS_ValidId(new_id) || init(f, old_id, 6) != ENCOS_OK) return ENCOS_INVALID;
    f->id = ENCOS_SETTINGS_ID;
    put16(f->data, old_id); f->data[3] = 4; put16(f->data + 4, new_id);
    return ENCOS_OK;
}
ENCOS_Result ENCOS_PackQueryId(ENCOS_Frame *f) {
    if (!f) return ENCOS_INVALID;
    memset(f, 0, sizeof(*f)); f->id = ENCOS_SETTINGS_ID; f->len = 4;
    f->data[0] = 0xFF; f->data[1] = 0xFF; f->data[3] = 0x82;
    return ENCOS_OK;
}
static size_t query_length(uint8_t code) {
    if ((code >= 1 && code <= 4) || (code >= 23 && code <= 28) ||
        (code >= 32 && code <= 34)) return 6;
    if (code == 5 || code == 22 || code == 31 || code == 38) return 4;
    if (code == 29 || code == 39) return 7;
    if (code == 30) return 8;
    if (code == 35 || code == 37) return 3;
    return 0;
}
ENCOS_Result ENCOS_Decode(const uint8_t *d, size_t len, const ENCOS_Ranges *r,
                         ENCOS_Feedback *out) {
    if (!d || !out || len < 2 || len > 8) return ENCOS_INVALID;
    ENCOS_Feedback f = {0};
    f.type = d[0] >> 5; f.error = d[0] & 0x1F;
    switch (f.type) {
    case 1:
        if (len != 8 || !ENCOS_ValidRanges(r)) return ENCOS_INVALID;
        f.position = decode_value(be16(d + 1), r->position, 65535);
        f.speed = decode_value(((uint16_t)d[3] << 4) | (d[4] >> 4), r->speed, 4095);
        f.current = decode_value(((uint16_t)(d[4] & 15) << 8) | d[5], r->current, 4095);
        f.motor_temp = ((float)d[6] - 50) / 2;
        f.mos_temp = ((float)d[7] - 50) / 2;
        f.valid = ENCOS_HAS_POSITION | ENCOS_HAS_SPEED | ENCOS_HAS_CURRENT |
                  ENCOS_HAS_MOTOR_TEMP | ENCOS_HAS_MOS_TEMP;
        break;
    case 2:
    case 3: {
        if (len != 8) return ENCOS_INVALID;
        float x = read_float(d + 1);
        if (!isfinite(x)) return ENCOS_INVALID;
        if (f.type == 2) { f.position = x * (ENCOS_PI / 180); f.valid = ENCOS_HAS_POSITION; }
        else { f.speed = x * (ENCOS_PI / 30); f.valid = ENCOS_HAS_SPEED; }
        if (!isfinite(f.position) || !isfinite(f.speed)) return ENCOS_INVALID;
        f.current = (float)signed16(d + 5) / 100;
        f.motor_temp = ((float)d[7] - 50) / 2;
        f.valid |= ENCOS_HAS_CURRENT | ENCOS_HAS_MOTOR_TEMP;
        break;
    }
    case 4:
        if (len != 3 || d[2] > 1) return ENCOS_INVALID;
        f.code = d[1]; f.status = d[2]; break;
    case 5: {
        size_t expected = query_length(d[1]);
        if (!expected) return ENCOS_UNSUPPORTED;
        if (len != expected) return ENCOS_INVALID;
        f.code = d[1]; f.payload_len = (uint8_t)(len - 2);
        memcpy(f.payload, d + 2, f.payload_len);
        if (f.code <= 4) {
            float x = read_float(d + 2);
            if (!isfinite(x)) return ENCOS_INVALID;
            if (f.code == 1) { f.position = x * (ENCOS_PI / 180); f.valid = ENCOS_HAS_POSITION; }
            if (f.code == 2) { f.speed = x * (ENCOS_PI / 30); f.valid = ENCOS_HAS_SPEED; }
            if (f.code == 3) { f.current = x; f.valid = ENCOS_HAS_CURRENT; }
            if (!isfinite(f.position) || !isfinite(f.speed)) return ENCOS_INVALID;
        }
        if (f.code == 37) {
            if (d[2] > 1) return ENCOS_INVALID;
            f.brake_released = d[2]; f.valid = ENCOS_HAS_BRAKE;
        }
        break;
    }
    case 6:
        if (len != 2 || d[1] > 1) return ENCOS_INVALID;
        f.brake_released = d[1]; f.valid = ENCOS_HAS_BRAKE; break;
    default: return ENCOS_UNSUPPORTED;
    }
    *out = f;
    return ENCOS_OK;
}
const char *ENCOS_ErrorString(uint8_t error) {
    static const char *const names[] = {
        "OK", "Motor overtemperature", "Overcurrent", "Overvoltage",
        "Undervoltage", "Encoder fault", "Brake overvoltage", "DRV fault"
    };
    return error < 8 ? names[error] : "Unknown ENCOS fault";
}
