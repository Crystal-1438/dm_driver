#include "bsp_supervise.h"
#include "bsp_time.h"

#if _RT_DEBUG_FRAMERATE
void FrameRateStatistics(FPS_t *FPS) {
    FPS->count++;
    FPS->now = BSP_sys_time_ms(); // NOLINT(bugprone-narrowing-conversions)
    int duration = FPS->now - FPS->last;
    if (duration >= 1000) {
        FPS->FPS = FPS->count * 1000 / duration;
        FPS->count = 0;
        FPS->last = FPS->now;
    }
}
#else
void FrameRateStatistics(FPS_t *FPS) {}
#endif