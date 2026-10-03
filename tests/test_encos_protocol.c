#include "encos_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const ENCOS_Ranges ranges = {
    {0, 500}, {0, 5}, {-12.5f, 12.5f}, {-18, 18}, {-30, 30}, {-30, 30}
}; /* Manual section 9.1.1 example only, NOT universal motor defaults. */

static void frame_is(const ENCOS_Frame *f, uint16_t id, const uint8_t *d, size_t len) {
    assert(f->id == id && f->len == len && memcmp(f->data, d, len) == 0);
}
static void near(float a, float b) { assert(fabsf(a - b) < 0.001f); }

int main(void) {
    ENCOS_Frame f;
    /* Golden vectors transcribed from V1.20EAP sections 9.1 and 11. */
    const uint8_t mixed[] = {0x01,0x47,0xFF,0x7F,0xFF,0x7F,0xF7,0xFF};
    assert(ENCOS_PackMixed(&f, 1, &ranges, 20, 5, 0, 0, 0) == ENCOS_OK);
    frame_is(&f, 1, mixed, 8);
    ENCOS_Ranges r50 = ranges; r50.kd.max = 50;
    assert(ENCOS_PackMixed(&f, 1, &r50, 20, 50, 0, 0, 0) == ENCOS_OK);
    frame_is(&f, 1, mixed, 8);
    assert(ENCOS_PackMixed(&f, 1, &ranges, 20, 2.5f, 0, 0, 0) == ENCOS_OK);
    assert((f.data[1] & 1) == 0 && f.data[2] == 255);
    assert(ENCOS_PackMixed(&f,1,&ranges,999,999,999,999,999) == ENCOS_OK);
    const uint8_t top[] = {0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    frame_is(&f,1,top,8);
    assert(ENCOS_PackMixed(&f,1,&ranges,-999,-999,-999,-999,-999) == ENCOS_OK);
    const uint8_t bottom[8] = {0}; frame_is(&f,1,bottom,8);
    const uint8_t pos[] = {0x20,0,0,0,0,0x7D,0x01,0x91};
    assert(ENCOS_PackPosition(&f, 1, 0, 50, 10, 1) == ENCOS_OK);
    frame_is(&f, 1, pos, 8);
    const uint8_t pos90[] = {0x28,0x56,0x80,0,0,0x32,0,0xCA};
    assert(ENCOS_PackPosition(&f, 1, 90, 20, 5, 2) == ENCOS_OK);
    frame_is(&f, 1, pos90, 8);
    const uint8_t neg90[] = {0x38,0x56,0x80,0,0,0x32,0,0xCA};
    assert(ENCOS_PackPosition(&f, 1, -90, 20, 5, 2) == ENCOS_OK);
    frame_is(&f, 1, neg90, 8);
    const uint8_t spd[] = {0x41,0x42,0x48,0,0,0,0x64};
    assert(ENCOS_PackSpeed(&f, 1, 50, 10, 1) == ENCOS_OK);
    frame_is(&f, 1, spd, 7);
    for (int mode = 0; mode <= 5; ++mode) {
        float value = mode == 2 ? 0 : mode == 5 ? 1 : 5;
        const uint8_t expected[] = {(uint8_t)(0x61 | (mode << 2)),
            (uint8_t)(mode == 2 || mode == 5 ? 0 : 1),
            (uint8_t)(mode == 2 ? 0 : mode == 5 ? 1 : 0xF4)};
        assert(ENCOS_PackCurrent(&f, 1, (ENCOS_CurrentMode)mode, value, 1) == ENCOS_OK);
        frame_is(&f, 1, expected, 3);
    }
    const uint8_t neg[] = {0x62,0xFE,0xD4};
    assert(ENCOS_PackCurrent(&f, 1, ENCOS_CURRENT, -3, 2) == ENCOS_OK);
    frame_is(&f, 1, neg, 3);
    const uint8_t zero[] = {0x01,0x23,0,3};
    assert(ENCOS_PackZero(&f, 0x123) == ENCOS_OK);
    frame_is(&f, 0x7FF, zero, 4);
    const uint8_t id[] = {0x01,0x23,0,4,0x07,0xFE};
    assert(ENCOS_PackSetId(&f, 0x123, 0x7FE) == ENCOS_OK);
    frame_is(&f, 0x7FF, id, 6);
    const uint8_t query_id[] = {0xFF,0xFF,0,0x82};
    assert(ENCOS_PackQueryId(&f) == ENCOS_OK);
    frame_is(&f, 0x7FF, query_id, 4);
    assert(ENCOS_PackQuery(&f, 1, 24) == ENCOS_OK);
    assert(f.len == 2 && f.data[0] == 0xE0 && f.data[1] == 0x18);

    ENCOS_Feedback fb;
    /* Non-symmetric current/torque ranges ensure these cannot be confused. */
    ENCOS_Ranges r = ranges; r.current.min = -10; r.current.max = 20;
    const uint8_t type1[] = {0x25,0xFF,0xFF,0,0x0F,0xFF,51,151};
    assert(ENCOS_Decode(type1, 8, &r, &fb) == ENCOS_OK);
    near(fb.position, 12.5f); near(fb.speed, -18); near(fb.current, 20);
    near(fb.motor_temp, 0.5f); near(fb.mos_temp, 50.5f); assert(fb.error == 5);
    const uint8_t type2[] = {0x40,0x42,0xB4,0,0,0xFE,0x0C,49}; /* 90deg, -5A */
    assert(ENCOS_Decode(type2, 8, &r, &fb) == ENCOS_OK);
    near(fb.position, ENCOS_PI/2); near(fb.current, -5); near(fb.motor_temp, -0.5f);
    assert(!(fb.valid & ENCOS_HAS_SPEED) && !(fb.valid & ENCOS_HAS_MOS_TEMP));
    const uint8_t type3[] = {0x60,0xC2,0x70,0,0,0,0x64,100}; /* -60rpm, 1A */
    assert(ENCOS_Decode(type3, 8, &r, &fb) == ENCOS_OK);
    near(fb.speed, -2*ENCOS_PI); assert(!(fb.valid & ENCOS_HAS_POSITION));
    const uint8_t type4[] = {0x80,1,1};
    assert(ENCOS_Decode(type4, 3, &r, &fb) == ENCOS_OK);
    assert(fb.type == 4 && fb.status == 1 && fb.valid == 0);
    const uint8_t type5[] = {0xA0,24,0,0,0,50};
    assert(ENCOS_Decode(type5, 6, &r, &fb) == ENCOS_OK);
    assert(fb.code == 24 && fb.payload_len == 4 && fb.payload[3] == 50 && !fb.valid);
    assert(fb.query_range.min == 0 && fb.query_range.max == 50);
    const uint8_t current_range[] = {0xA0,28,0xFF,0x9C,0,0xC8};
    assert(ENCOS_Decode(current_range,6,&r,&fb) == ENCOS_OK);
    near(fb.query_range.min,-10); near(fb.query_range.max,20);
    const uint8_t query_pos[] = {0xA0,1,0x42,0xB4,0,0};
    assert(ENCOS_Decode(query_pos, 6, &r, &fb) == ENCOS_OK);
    near(fb.position, ENCOS_PI/2);
    const uint8_t type6[] = {0xC0,1};
    assert(ENCOS_Decode(type6, 2, &r, &fb) == ENCOS_OK);
    assert(fb.brake_released == 1 && fb.valid == ENCOS_HAS_BRAKE);

    /* Rejected frames do not modify output; all truncated/overlong types rejected. */
    ENCOS_Feedback before = fb;
    for (size_t n = 0; n <= 9; ++n) if (n != 8) {
        assert(ENCOS_Decode(type1, n, &r, &fb) == ENCOS_INVALID);
        assert(memcmp(&before, &fb, sizeof(fb)) == 0);
    }
    const uint8_t nan_reply[] = {0x40,0x7F,0xC0,0,0,0,0,100};
    assert(ENCOS_Decode(nan_reply, 8, &r, &fb) == ENCOS_INVALID);
    assert(ENCOS_PackMixed(&f, 1, &r, NAN, 0, 0, 0, 0) == ENCOS_INVALID);
    assert(ENCOS_PackMixed(&f, 0x7FF, &r, 0, 0, 0, 0, 0) == ENCOS_INVALID);
    r.kd.max = 0;
    assert(ENCOS_PackMixed(&f, 1, &r, 0, 0, 0, 0, 0) == ENCOS_INVALID);
    assert(ENCOS_PackCurrent(&f, 1, ENCOS_CURRENT, INFINITY, 1) == ENCOS_INVALID);
    assert(ENCOS_PackCurrent(&f, 1, ENCOS_MECHANICAL_BRAKE, 2, 1) == ENCOS_INVALID);
    assert(ENCOS_PackPosition(&f, 1, 0, -1, 1, 1) == ENCOS_INVALID);
    assert(ENCOS_PackSpeed(&f, 1, 0, 1, 4) == ENCOS_INVALID);
    assert(ENCOS_PackQuery(&f, 1, 36) == ENCOS_UNSUPPORTED);
    /* Every supported packet family rejects truncated and overlong frames. */
    const uint8_t *packets[] = {type1,type2,type3,type4,type5,type6};
    const size_t lengths[] = {8,8,8,3,6,2};
    for (size_t i=0;i<6;++i) {
        uint8_t buffer[9] = {0}; memcpy(buffer,packets[i],lengths[i]);
        for (size_t n=0;n<=9;++n) if (n!=lengths[i])
            assert(ENCOS_Decode(buffer,n,&ranges,&fb) != ENCOS_OK);
    }
    puts("ENCOS protocol tests passed");
    return 0;
}
