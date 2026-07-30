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

#define AGR_FRAME_DLC 4
#define AGR_NO_RETURN 0xFFFF
#define AGR_RX_RING 4
#define AGR_CAL_ABORT_NET_M 0.3f

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
    volatile AgrRawSample rx[AGR_RX_RING];
    volatile uint8_t rx_head;
    uint8_t rx_tail;
    volatile time_t last_rx_tick;
    uint8_t last_counter;
    uint32_t can_loss_count;

    bool sim_active;
    float sim_grade_tan;
    uint8_t sim_phase;

    AgrPitchRing pitch_ring;
    AgrGeometry geo;
    AgrTuning tuning;
    AgrProfile profile;
    AgrTrust trust;
    AgrCond cond;
    AgrFit fit;

    float trim_deg;
    float lag_ticks;
    uint32_t timeout_ticks;
    float sys_to_main;
    float last_distance;
    bool have_distance;

    AgrCalMode cal_mode;
    float cal_net_m;
    bool cal_sweep_applied;
    AgrCalSweep cal_sweep;
    AgrTauScan cal_tau;
    AgrCalResult cal_result;
    uint32_t cal_tau_ticks;

    float g_cmd_deg;
    float fade;
    float gates_f;
    float fit_residual_m;
    float fit_weight;
    float valid_fraction;
    float far_chord_deg;
    float law_raw_deg;
    float setpoint;

    float node_status;
    float node_gate_pct;
    float node_sat;
    float node_temp_c;
    float node_cfg_crc;
} AGR;

void agr_init(AGR *agr);
void agr_reset(AGR *agr);
void agr_configure(AGR *agr, const RefloatConfig *config, float frequency);
void agr_update(
    AGR *agr,
    const MotorData *motor,
    const IMU *imu,
    const Time *time,
    const RefloatConfig *config,
    float dt
);
void agr_winddown(AGR *agr);
bool agr_handle_can_frame(AGR *agr, const uint8_t *data, uint8_t len);
void agr_handle_health_frame(AGR *agr, const uint8_t *data, uint8_t len);
