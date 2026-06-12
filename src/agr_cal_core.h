// forks/refloat/src/agr_cal_core.h — pure, host-testable calibration core
#pragma once
#include "agr_geometry.h"

#include <stdbool.h>
#include <stdint.h>

#define AGR_CAL_BIN_DEG 0.5f
#define AGR_CAL_BINS 160  // pitch -40..+40 deg
#define AGR_CAL_MIN_N 5
#define AGR_CAL_MIN_DELTA_DEG 8.0f
#define AGR_CAL_MIN_SPAN_DEG 15.0f
#define AGR_CAL_MAX_RESID_M 0.015f
#define AGR_CAL_GN_ITERS 8

typedef struct {
    float sum_r;   // m
    float sum_r2;  // m^2
    uint16_t n;
} AgrCalBin;

typedef struct {
    AgrCalBin bin[AGR_CAL_BINS];
    bool active;
} AgrCalSweep;

typedef enum {
    AGR_CAL_OK = 0,
    AGR_CAL_COVERAGE,  // pitch span too small
    AGR_CAL_RESIDUAL,  // fit converged but residual over threshold
    AGR_CAL_SINGULAR,  // normal equations singular
} AgrCalStatus;

typedef struct {
    AgrCalStatus status;
    float mount_height, mount_fwd;  // m
    float mount_offset_deg;         // residual vs the CAD prior
    float range_bias;               // m
    float rms_m;
    float span_deg;  // pitch coverage actually used
    int bins_used;
} AgrCalResult;

void agr_cal_sweep_init(AgrCalSweep *s);
void agr_cal_sweep_add(AgrCalSweep *s, float pitch_rad, float range_m);
// mount_angle_cad in rad; fits {h, f, theta_err, b} to the binned curve
AgrCalResult agr_cal_fit(const AgrCalSweep *s, float mount_angle_cad);
// predicted flat-ground slant range for params (shared with tau scan + dump)
float agr_cal_model(
    float pitch, float mount_angle, float theta_err, float h, float f, float b
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
// feed one range sample; ring/geometry provide pitch-at-lag lookup.
// tick_hz: pitch ring write rate (500); used to convert ms to ticks.
void agr_tau_feed(
    AgrTauScan *t, const AgrPitchRing *ring, const AgrGeometry *geo, float range_m,
    float tick_hz
);
// winner with parabolic interpolation; returns ms, or -1 if n too small
float agr_tau_result(const AgrTauScan *t);

// live trim: slow integrator on trusted grade bias; returns updated trim (deg)
float agr_trim_update(float trim_deg, float g_cmd_deg, float fade, bool moving, float dt);
