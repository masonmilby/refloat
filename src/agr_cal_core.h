#pragma once
#include "agr_geometry.h"

#include <stdbool.h>
#include <stdint.h>

#define AGR_CAL_BIN_DEG 0.5f
#define AGR_CAL_BINS 160
#define AGR_CAL_MIN_N 5
#define AGR_CAL_MIN_DELTA_DEG 8.0f
#define AGR_CAL_MIN_SPAN_DEG 15.0f
#define AGR_CAL_MAX_RESID_M 0.015f
#define AGR_CAL_GN_ITERS 8

typedef struct {
    float sum_r_m;
    float sum_r2_m2;
    uint16_t n;
} AgrCalBin;

typedef struct {
    AgrCalBin bin[AGR_CAL_BINS];
    bool active;
} AgrCalSweep;

typedef enum {
    AGR_CAL_OK = 0,
    AGR_CAL_COVERAGE,
    AGR_CAL_RESIDUAL,
    AGR_CAL_SINGULAR,
} AgrCalStatus;

typedef struct {
    AgrCalStatus status;
    float mount_height_m, mount_fwd_m;
    float mount_offset_deg;
    float range_bias_m;
    float rms_m;
    float span_deg;
    int bins_used;
} AgrCalResult;

void agr_cal_sweep_init(AgrCalSweep *s);
void agr_cal_sweep_add(AgrCalSweep *s, float pitch_rad, float range_m);
AgrCalResult agr_cal_fit(const AgrCalSweep *s, float mount_angle_cad, float wheel_radius_m);
float agr_cal_model(
    float pitch,
    float mount_angle_rad,
    float theta_err,
    float h,
    float f,
    float b,
    float wheel_radius_m
);

#define AGR_TAU_CANDIDATES 16
#define AGR_TAU_STEP_MS 4.0f
#define AGR_TRIM_LIMIT_DEG 0.5f
#define AGR_TRIM_TAU_S 600.0f

typedef struct {
    float sse[AGR_TAU_CANDIDATES];
    uint32_t n;
    bool active;
} AgrTauScan;

void agr_tau_init(AgrTauScan *t);
void agr_tau_feed(
    AgrTauScan *t, const AgrPitchRing *ring, const AgrGeometry *geo, float range_m, float tick_hz
);
float agr_tau_result(const AgrTauScan *t);

float agr_trim_update(float trim_deg, float g_cmd_deg, float fade, bool moving, float dt);
