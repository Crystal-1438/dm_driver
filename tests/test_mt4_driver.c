#include "MT4_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t clock_ms;
static int alloc_fail_after = -1;
void *test_alloc(size_t n) {
    if (alloc_fail_after == 0) return NULL;
    if (alloc_fail_after > 0) --alloc_fail_after;
    return malloc(n);
}
uint32_t BSP_sys_time_ms(void) { return clock_ms; }
cvector *cvector_create(size_t size) {
    cvector *v = test_alloc(sizeof(*v));
    if (v) { v->cv_len = 0; v->size = size; v->data = NULL; }
    return v;
}
void *cvector_val_at(cvector *v, size_t i) { return (char *)v->data + i * v->size; }
void cvector_pushback(cvector *v, void *p) {
    void *next = realloc(v->data, (v->cv_len + 1) * v->size); assert(next);
    v->data = next; memcpy(cvector_val_at(v, v->cv_len++), p, v->size);
}
void FrameRateStatistics(FPS_t *f) { f->count++; }
static rx_callback callbacks[2];
static unsigned registered[2], filter_count;
static struct { uint8_t bus; ENCOS_Frame f; } sent[256];
static size_t sent_count;
void BSP_CAN_Send(uint8_t bus, uint32_t id, uint8_t *data, basic_data_t len) {
    assert(sent_count < 256 && len <= 8);
    sent[sent_count].bus = bus;
    sent[sent_count].f.id = (uint16_t)id; sent[sent_count].f.len = (uint8_t)len;
    memcpy(sent[sent_count++].f.data, data, len);
}
void BSP_CAN_RegisterRxCallback(uint8_t bus, rx_callback cb) {
    callbacks[bus] = cb; registered[bus]++;
}
void BSP_CAN_AddFilter(uint8_t bus, uint32_t id) {
    assert(bus < 2 && id > 0 && id <= 0x7FF); ++filter_count;
}
static int control_calls;
static void control(MT4_motor *m) { (void)m; ++control_calls; }
static MT4_motor_config config(uint8_t bus, uint16_t id) {
    MT4_motor_config c = {0};
    c.bsp_can_index = bus; c.can_rx_id = id; c.can_tx_id = id;
    c.ranges = (ENCOS_Ranges){{0,500},{0,5},{-12.5f,12.5f},{-18,18},{-30,30},{-10,10}};
    c.position_offset = 1;
    return c;
}
static void telemetry(uint8_t bus, uint16_t id) {
    uint8_t data[] = {0x20,0xFF,0xFF,0x7F,0xFF,0xFF,101,151};
    callbacks[bus](bus, id, data, sizeof(data));
}
static void is_stop(size_t i, uint8_t first) {
    assert(sent[i].f.len == 3 && sent[i].f.data[0] == first &&
           sent[i].f.data[1] == 0 && sent[i].f.data[2] == 0);
}
int main(void) {
    MT4motor_Calc_Send(); /* safe before init */
    assert(MT4motor_Create(NULL, NULL) == NULL);
    MT4_motor_config invalid = config(0, 1); invalid.ranges.kd.max = 0;
    assert(MT4motor_Create(&invalid, NULL) == NULL);
    invalid = config(2, 1); assert(MT4motor_Create(&invalid, NULL) == NULL);
    invalid = config(0, 0x7FF); assert(MT4motor_Create(&invalid, NULL) == NULL);
    invalid = config(0, 1); invalid.can_rx_id = 2;
    assert(MT4motor_Create(&invalid, NULL) == NULL);
    MT4_motor_config ca = config(0, 0x123), cb = config(1, 0x123);
    alloc_fail_after = 0; assert(!MT4motor_Create(&ca, control)); alloc_fail_after = -1;
    MT4motor_Driver_Init();
    alloc_fail_after = 1; assert(!MT4motor_Create(&ca, control)); alloc_fail_after = -1;
    MT4_motor *a = MT4motor_Create(&ca, control); assert(a);
    cb.torque_constant = 2; cb.stop_mode = mt4_damping_stop;
    MT4_motor *b = MT4motor_Create(&cb, control); assert(b);
    assert(!MT4motor_Create(&ca, control)); /* same ID on another bus allowed, duplicate rejected */
    assert(registered[0] == 1 && registered[1] == 1 && filter_count == 4);
    assert(MT4motor_Enable(a) == ENCOS_BUSY && !MT4motor_Is_Online(a));
    MT4motor_Calc_Send();
    assert(sent_count == 2 && !control_calls); is_stop(0,0x61); is_stop(1,0x69);
    telemetry(0,0x123);
    assert(MT4motor_Is_Online(a) && !MT4motor_Is_Online(b));
    assert(a->update && a->fdb_position == 12.5f && a->real_fdb_position == 13.5f);
    assert(a->fdb_current == 10 && isnan(a->fdb_torque) && !a->torque_valid);
    assert(a->t_rotor == 25.5f && a->t_mos == 50.5f);
    a->ref_position = 13.5f; assert(MT4Motor_Check_InPlace(a,0));
    telemetry(1,0x123); assert(b->torque_valid && b->fdb_torque == 20);
    uint32_t before_rx = a->last_rx_ms; float before_pos = a->fdb_position;
    clock_ms = 10;
    uint8_t malformed[] = {0x20,0,0};
    callbacks[0](0,0x123,malformed,3);
    assert(a->last_rx_ms == before_rx && a->fdb_position == before_pos);
    uint8_t ack[] = {0x80,1,1}; callbacks[0](0,0x123,ack,3);
    assert(a->last_rx_ms == 10 && a->fdb_position == before_pos && a->last_position_ms == 0);
    a->kp = 20; a->kd = 5; a->ref_position = 1; /* offset cancels to motor pos 0 */
    assert(MT4motor_Enable(a) == ENCOS_OK);
    size_t n = sent_count; MT4motor_Calc_Send();
    const uint8_t mixed[] = {1,0x47,0xFF,0x7F,0xFF,0x7F,0xF7,0xFF};
    assert(sent_count == n+2 && sent[n].f.len == 8 && !memcmp(sent[n].f.data,mixed,8));
    assert(control_calls == 1);
    /* Old implementation injected an extra enable/damping frame every 300ms. */
    clock_ms = 350; telemetry(0,0x123); telemetry(1,0x123);
    n = sent_count; MT4motor_Calc_Send(); assert(sent_count == n+2);
    clock_ms = 451; n = sent_count; MT4motor_Calc_Send();
    assert(a->enable == mt4_stop && !MT4motor_Is_Online(a)); is_stop(n,0x61);
    telemetry(0,0x123); n = sent_count; MT4motor_Calc_Send(); is_stop(n,0x61);
    assert(a->enable == mt4_stop); /* no automatic restart after link recovery */
    assert(MT4motor_Enable(a) == ENCOS_OK);
    a->kd = NAN; n = sent_count; MT4motor_Calc_Send();
    assert(a->enable == mt4_stop && a->last_result == ENCOS_INVALID); is_stop(n,0x61);
    a->kd = 5;
    assert(MT4motor_Query(a,24) == ENCOS_OK);
    assert(sent[sent_count-1].f.data[0] == 0xE0 && sent[sent_count-1].f.data[1] == 0x18);
    assert(MT4motor_Enable(a) == ENCOS_OK);
    assert(MT4motor_Save_Zero(a) == ENCOS_BUSY);
    assert(MT4motor_Disable(a) == ENCOS_OK);
    n = sent_count;
    assert(MT4motor_Save_Zero(a) == ENCOS_OK);
    const uint8_t zero[] = {1,0x23,0,3};
    assert(sent[n].f.id == 0x7FF && sent[n].f.len == 4 && !memcmp(sent[n].f.data,zero,4));
    assert(MT4motor_Query(a,1) == ENCOS_BUSY && MT4motor_Enable(a) == ENCOS_BUSY);
    n = sent_count; clock_ms = 951; MT4motor_Calc_Send();
    assert(sent_count == n+1 && sent[n].bus == 1); /* exactly 500ms still guarded */
    uint8_t zero_ack[] = {1,0x23,1,3}; callbacks[0](0,0x7FF,zero_ack,4);
    assert(a->zero_ack_received && a->zero_ack_success && !a->zero_pending);
    assert(!(a->feedback_valid & ENCOS_HAS_POSITION));
    clock_ms = 952; n = sent_count; MT4motor_Calc_Send(); assert(sent_count == n+2);
    assert(MT4motor_Enable(a) == ENCOS_BUSY); telemetry(0,0x123);
    assert(MT4motor_Enable(a) == ENCOS_OK);
    /* Fault feedback stops normal output, and unknown commands emit nothing. */
    uint8_t fault[] = {0xC7,1}; callbacks[0](0,0x123,fault,2);
    n = sent_count; MT4motor_Calc_Send(); is_stop(n,0x61);
    assert(a->enable == mt4_stop && MT4motor_Enable(a) == ENCOS_BUSY);
    n = sent_count;
    assert(MT4motor_Clear_Err(a) == ENCOS_UNSUPPORTED);
    assert(MT4motor_Reset_Round(a) == ENCOS_UNSUPPORTED);
    assert(MT4motor_Broadcast_Cmd(a,4) == ENCOS_UNSUPPORTED && sent_count == n);
    /* Unsigned millisecond wraparound. */
    clock_ms = UINT32_MAX-20; telemetry(0,0x123);
    clock_ms = 10; assert(MT4motor_Is_Online(a));
    clock_ms = 100; assert(!MT4motor_Is_Online(a));
    /* Same-bus peers block zeroing while active; a setting quiets ALL peers. */
    MT4_motor_config cc = config(0,0x124);
    MT4_motor *peer = MT4motor_Create(&cc,control); assert(peer);
    telemetry(0,0x124); assert(MT4motor_Enable(peer) == ENCOS_OK);
    assert(MT4motor_Save_Zero(a) == ENCOS_BUSY);
    assert(MT4motor_Disable(peer) == ENCOS_OK);
    n = sent_count; assert(MT4motor_Save_Zero(a) == ENCOS_OK);
    MT4motor_Calc_Send();
    assert(sent_count == n+2 && sent[n+1].bus == 1);
    /* Reply must be from correct bus/ID, and late (600ms) ACK is accepted. */
    uint8_t failed_zero[] = {1,0x23,1,0};
    callbacks[1](1,0x7FF,failed_zero,4); assert(!a->zero_ack_received);
    clock_ms = 700; callbacks[0](0,0x7FF,failed_zero,4);
    assert(a->zero_ack_received && !a->zero_ack_success && !a->zero_pending);
    assert(MT4motor_Save_Zero(a) == ENCOS_OK);
    clock_ms = 1700; MT4motor_Calc_Send();
    assert(!a->zero_pending && !a->zero_ack_received && !a->zero_ack_success);
    /* Decimation does not generate hidden enable frames or run stopped PID. */
    peer->config.fre_rel = 3; peer->frequency_cnt = 0;
    n = sent_count; MT4motor_Calc_Send(); MT4motor_Calc_Send();
    assert(sent_count == n+4);
    MT4motor_Calc_Send(); assert(sent_count == n+7);
    extern cvector *mt4_motor_list, *monitor_lists;
    for (size_t i=0;i<mt4_motor_list->cv_len;++i) {
        MT4_motor *m=*(MT4_motor **)cvector_val_at(mt4_motor_list,i);
        free(m->motor_controller); free(m);
    }
    for (size_t i=0;i<monitor_lists->cv_len;++i) free(*(monitor_item **)cvector_val_at(monitor_lists,i));
    free(mt4_motor_list->data); free(mt4_motor_list);
    free(monitor_lists->data); free(monitor_lists);
    puts("ENCOS MT4 driver tests passed");
    return 0;
}
