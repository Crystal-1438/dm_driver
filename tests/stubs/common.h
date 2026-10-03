#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
void *test_alloc(size_t size);
#define RT_MALLOC test_alloc
#define RT_FREE free
#define _MID(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
static inline float fsgn(float x) { return x > 0 ? 1.0f : x < 0 ? -1.0f : 0.0f; }
