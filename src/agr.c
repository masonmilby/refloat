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

#include "agr.h"

#include "conf/buffer.h"
#include "lib/utils.h"

#include <math.h>

void agr_init(AGR *agr) {
    agr->mb_surface_angle = 0.0f;
    agr->mb_quality = 0.0f;
    agr->mb_seq = 0;
    agr->mb_diag = 0;
    agr->mb_rx_tick = 0;

    agr->sim_active = false;
    agr->sim_surface_angle = 0.0f;
    agr->sim_quality = 0.0f;

    ema_init(&agr->offset_ema);

    agr_reset(agr);
}

void agr_reset(AGR *agr) {
    agr->trusted = false;
    agr->fade = 0.0f;
    agr->dir_forward = true;
    agr->grade = 0.0f;
    ema_reset(&agr->offset_ema, 0.0f);
    agr->setpoint = 0.0f;
}

void agr_configure(AGR *agr, const RefloatConfig *config, float frequency) {
    ema_configure(&agr->offset_ema, config->agr_filter, frequency);
}

void agr_update(
    AGR *agr,
    const MotorData *motor,
    const IMU *imu,
    const Time *time,
    const RefloatConfig *config,
    float dt
) {
    // bench sim: refresh the mailbox each tick so staleness sees a live sensor
    if (agr->sim_active) {
        agr->mb_surface_angle = agr->sim_surface_angle;
        agr->mb_quality = agr->sim_quality;
        agr->mb_rx_tick = time->now;
    }

    // mailbox snapshot; torn reads benign (see agr.h)
    float surface_angle = agr->mb_surface_angle;
    float quality = agr->mb_quality;
    bool stale = time->now - agr->mb_rx_tick > AGR_TIMEOUT_TICKS;

    agr->surface_angle = surface_angle;
    agr->quality = quality;

    // trust gate: staleness + quality hysteresis (holds between thresholds)
    if (stale || quality < config->agr_quality_off) {
        agr->trusted = false;
    } else if (quality >= config->agr_quality_on) {
        agr->trusted = true;
    }

    // ease the trust transition, never snap
    rate_limitf(&agr->fade, agr->trusted ? 1.0f : 0.0f, config->agr_fade_rate * dt);

    // derotate to gravity: deck attitude cancels between imu->pitch and
    // surface_angle, leaving ground-vs-gravity (self-tilt feeds back nothing)
    agr->grade = surface_angle + imu->pitch + config->agr_mount_offset;

    // debounced travel direction (+-100 erpm)
    if (motor->erpm > 100.0f) {
        agr->dir_forward = true;
    } else if (motor->erpm < -100.0f) {
        agr->dir_forward = false;
    }

    // offset is a pure function of the current grade: up/down picks the gain,
    // the grade's sign sets the direction, so no per-case mode logic is needed
    bool uphill = agr->dir_forward == (agr->grade > 0.0f);
    float strength = uphill ? config->agr_strength_up : config->agr_strength_down;
    float raw = clampf(
        strength * agr->grade, -config->agr_angle_limit, config->agr_angle_limit
    );

    // optional low-speed taper; 0 disables so the offset holds at a standstill
    if (config->agr_taper_erpm > 0) {
        raw *= fminf(motor->abs_erpm / config->agr_taper_erpm, 1.0f);
    }

    // conditioning: fade, low-pass, rate cap; reject spurious frames and jitter
    ema_update(&agr->offset_ema, raw * agr->fade);
    rate_limitf(&agr->setpoint, agr->offset_ema.value, config->agr_rate_limit * dt);
}

void agr_winddown(AGR *agr) {
    agr->setpoint *= 0.995f;
    agr->offset_ema.value *= 0.995f;
}

bool agr_handle_can_frame(AGR *agr, const uint8_t *data, uint8_t len) {
    if (len < AGR_FRAME_DLC) {
        return false;
    }

    int32_t ind = 0;
    int16_t angle_cdeg = buffer_get_int16(data, &ind);

    agr->mb_surface_angle = angle_cdeg / AGR_ANGLE_SCALE;
    agr->mb_quality = data[ind] / AGR_QUALITY_SCALE;
    agr->mb_seq = data[ind + 1];
    agr->mb_diag = data[ind + 2];
    agr->mb_rx_tick = vesc_system_time_ticks();

    return true;
}
