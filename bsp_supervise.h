#pragma once

#pragma pack(1)
typedef struct {
    int count;
    int FPS;
    int now;
    int last;
} FPS_t;

#pragma pack()
#ifdef __cplusplus
extern "C" {
#endif

void FrameRateStatistics(FPS_t *FPS);
#ifdef __cplusplus
}
#endif
