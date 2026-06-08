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

#include "conf/datatypes.h"
#include "filters/ema.h"
#include "imu.h"
#include "motor_data.h"
#include "time.h"

#include <stdbool.h>
#include <stdint.h>

// Grade frame received over CAN, big-endian (VESC buffer_* convention).
#define AGR_FRAME_DLC 8
#define AGR_ANGLE_SCALE 100.0f  // int16 centidegrees
#define AGR_QUALITY_SCALE 255.0f  // uint8 -> 0..1
// no frame for 50 ms counts as link loss
#define AGR_TIMEOUT_TICKS (SYSTEM_TICK_RATE_HZ / 20)

typedef struct {
    // mailbox: written by the CAN rx callback, read by agr_update on the main
    // thread. torn reads mix adjacent frames' fields; benign, all conditioned.
    volatile float mb_surface_angle;  // degrees, sensor frame
    volatile float mb_quality;  // 0..1
    volatile uint8_t mb_seq;
    volatile uint8_t mb_diag;
    volatile time_t mb_rx_tick;

    // bench sim (agr_sim terminal command)
    bool sim_active;
    float sim_surface_angle;
    float sim_quality;

    bool trusted;
    float fade;  // 0..1, slews at agr_fade_rate

    bool dir_forward;  // debounced travel direction (+-100 erpm hysteresis)

    // telemetry mirrors of the last consumed wire values
    float surface_angle;
    float quality;

    float grade;  // derotated ground-vs-gravity, degrees
    EMA offset_ema;
    float setpoint;  // conditioned offset, added into the tilt aggregation
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

// decode one contract frame into the mailbox; returns false on a short frame
bool agr_handle_can_frame(AGR *agr, const uint8_t *data, uint8_t len);
