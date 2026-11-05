// API for EqApo.c

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// load/reset/blabla u get it
void eq_reset(void);
bool eq_load_file(const char *path);
int eq_filter_count(void);
void eq_set_enabled(bool on);
bool eq_enabled(void);

void eq_apply_int32_stereo(int32_t *stereo, size_t frames, uint32_t rate);

void eq_apply_float(int channel, float *samples, size_t count, uint32_t rate);
