// host-compat shim: the subset of the VESC package API that the AGR sources
// reference, so lib/, time.h and agr.c link under host gcc
#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// agr's time.h typedefs its own systime_t-sized time_t; predefine
// glibc's guard so <stdlib.h>'s transitive time_t typedef is skipped
// instead of conflicting with it in host test builds.
#define __time_t_defined 1

#define SYSTEM_TICK_RATE_HZ 10000

typedef uint32_t systime_t;
typedef int mc_fault_code;

#define CFG_PARAM_si_wheel_diameter 0

typedef struct {
    bool (*app_is_output_disabled)(void);
    float (*system_time)(void);
    int (*printf)(const char *fmt, ...);
    float (*get_cfg_float)(int param);
} vesc_if_host;

extern vesc_if_host *VESC_IF;

// test-settable clock: vesc_system_time_ticks() returns this
extern systime_t host_now_ticks;
