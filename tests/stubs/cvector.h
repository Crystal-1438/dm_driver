#pragma once
#include "common.h"
typedef struct { size_t cv_len, size; void *data; } cvector;
cvector *cvector_create(size_t size);
void *cvector_val_at(cvector *v, size_t i);
void cvector_pushback(cvector *v, void *p);
