// Copyright 2026 Mason Milburn
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include "agr_cal_core.h"
#include "agr_control.h"
#include "agr_geometry.h"
#include "agr_profile.h"
#include "conf/datatypes.h"
#include "imu.h"
#include "motor_data.h"
#include "time.h"

#include <stdbool.h>
#include <stdint.h>

// esp_tof range frame: base+0, DLC 4, big-endian (ESP_ToF_SPEC.md section 5.1)
#define AGR_FRAME_DLC 4
#define AGR_NO_RETURN 0xFFFF
#define AGR_TIMEOUT_TICKS (SYSTEM_TICK_RATE_HZ / 20)  // 50 ms: silence is loss
#define AGR_RX_RING 4

typedef struct {
    uint16_t range_mm;
    uint8_t strength;
    uint8_t counter;
    time_t rx_tick;
} AgrRawSample;

typedef enum {
    AGR_CAL_IDLE = 0,
    AGR_CAL_SWEEPING,
    AGR_CAL_TAU,
    AGR_CAL_PENDING,
    AGR_CAL_TAU_PENDING,
} AgrCalMode;

typedef struct {
    // rx ring: written by the CAN callback, drained by agr_update (single
    // writer / single reader; torn reads benign as in v1)
    volatile AgrRawSample rx[AGR_RX_RING];
    volatile uint8_t rx_head;
    uint8_t rx_tail;
    volatile time_t last_rx_tick;
    uint8_t last_counter;
    uint32_t can_loss_count;

    bool sim_active;
    float sim_grade;  // tan units
    uint8_t sim_phase;

    AgrPitchRing pitch_ring;
    AgrGeometry geo;  // radians/meters, derived from config + trim each configure
    AgrTuning tuning;
    AgrProfile profile;
    AgrTrust trust;
    AgrCond cond;
    AgrFit fit;

    float trim_deg;
    float lag_ticks;
    float sys_to_main;  // SYSTEM_TICK_RATE_HZ ticks → main-loop ticks
    float last_distance;
    bool have_distance;

    // calibration session state
    AgrCalMode cal_mode;
    bool cal_sweep_applied;  // true once a sweep has been applied this boot
    AgrCalSweep cal_sweep;
    AgrTauScan cal_tau;
    AgrCalResult cal_result;
    uint32_t cal_tau_ticks;

    // telemetry mirrors (floats for rt_data)
    float g_cmd;         // fitted grade, deg
    float fade;          // == trust.fade
    float gates_f;       // bitfield: link|sight<<1|fit<<2|policy<<3
    float fit_residual;  // m
    float fit_weight;
    float valid_fraction;
    float far_chord;  // deg, 0 when inactive
    float setpoint;   // == cond.setpoint

    // node health mirrors (diagnostics only — never read in control path)
    float node_status;    // health byte 0 (status bits)
    float node_gate_pct;  // byte 1
    float node_sat;       // byte 2
    float node_temp_c;    // byte 4 minus 64
    float node_cfg_crc;   // byte 6
} AGR;

void agr_init(AGR *agr);
void agr_reset(AGR *agr);
void agr_configure(AGR *agr, const RefloatConfig *config, float frequency);
void agr_update(
    AGR *agr, const MotorData *motor, const IMU *imu, const Time *time,
    const RefloatConfig *config, float dt
);
void agr_winddown(AGR *agr);
bool agr_handle_can_frame(AGR *agr, const uint8_t *data, uint8_t len);
void agr_handle_health_frame(AGR *agr, const uint8_t *data, uint8_t len);
