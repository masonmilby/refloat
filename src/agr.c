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

#include "agr_math.h"
#include "conf/buffer.h"
#include "lib/utils.h"

#include <math.h>

void agr_init(AGR *agr) {
    agr->rx_head = 0;
    agr->rx_tail = 0;
    agr->last_rx_tick = 0;
    agr->last_counter = 0;
    agr->can_loss_count = 0;
    agr->sim_active = false;
    agr->sim_grade = 0.0f;
    agr->sim_phase = 0;
    agr_pitch_ring_init(&agr->pitch_ring);
    agr->sys_to_main = 500.0f / (float) SYSTEM_TICK_RATE_HZ;
    agr->trim_deg = 0.0f;
    agr->have_distance = false;
    agr->last_distance = 0.0f;
    agr->cal_mode = AGR_CAL_IDLE;
    agr->cal_sweep_applied = false;
    agr->node_status = 0.0f;
    agr->node_gate_pct = 0.0f;
    agr->node_sat = 0.0f;
    agr->node_temp_c = 0.0f;
    agr->node_cfg_crc = 0.0f;
    agr_reset(agr);
}

void agr_reset(AGR *agr) {
    agr->sim_active = false;
    agr_profile_init(&agr->profile);
    agr_trust_init(&agr->trust);
    agr_cond_reset(&agr->cond);
    agr->fit = (AgrFit){0};
    agr->g_cmd = 0.0f;
    agr->fade = 0.0f;
    agr->gates_f = 0.0f;
    agr->fit_residual = 0.0f;
    agr->fit_weight = 0.0f;
    agr->valid_fraction = 0.0f;
    agr->far_chord = 0.0f;
    agr->law_raw = 0.0f;
    agr->setpoint = 0.0f;
}

void agr_configure(AGR *agr, const RefloatConfig *config, float frequency) {
    agr->geo.mount_angle = config->agr_mount_angle * AGR_DEG2RAD;
    agr->geo.mount_offset = (config->agr_mount_offset + agr->trim_deg) * AGR_DEG2RAD;
    agr->geo.mount_height = config->agr_mount_height;
    agr->geo.mount_fwd = config->agr_mount_fwd;
    agr->geo.range_bias = config->agr_range_bias;
    agr->geo.wheel_radius = VESC_IF->get_cfg_float(CFG_PARAM_si_wheel_diameter) * 0.5f;
    agr->lag_ticks = config->agr_lag_ms * frequency / 1000.0f;
    agr->sys_to_main = frequency / (float) SYSTEM_TICK_RATE_HZ;
    agr->timeout_ticks =
        (uint32_t) (config->agr_stale_ms * 0.001f * (float) SYSTEM_TICK_RATE_HZ);

    agr->tuning.strength_up = config->agr_strength_up;
    agr->tuning.strength_down = config->agr_strength_down;
    agr->tuning.angle_limit_up = config->agr_angle_limit_up;
    agr->tuning.angle_limit_down = config->agr_angle_limit_down;
    agr->tuning.taper_erpm = (float) config->agr_taper_erpm;
    agr->tuning.sight_on = config->agr_sight_on;
    agr->tuning.sight_off = config->agr_sight_off;
    agr->tuning.residual_max = config->agr_residual_max;
    agr->tuning.fade_rate = config->agr_fade_rate;
    agr->tuning.reverse_fade_m = config->agr_reverse_fade_m;
    agr->tuning.dir_flip_m = config->agr_dir_flip_m;
    agr->tuning.rearm_m = config->agr_rearm_m;
    agr->tuning.rate_limit = config->agr_rate_limit;
    agr_cond_configure(&agr->cond, config->agr_filter, frequency);
}

bool agr_handle_can_frame(AGR *agr, const uint8_t *data, uint8_t len) {
    if (len < AGR_FRAME_DLC) {
        return false;
    }
    int32_t ind = 0;
    uint8_t next = (uint8_t) ((agr->rx_head + 1) % AGR_RX_RING);
    AgrRawSample s;
    s.range_mm = buffer_get_uint16(data, &ind);
    s.strength = data[ind];
    s.counter = data[ind + 1];
    s.rx_tick = vesc_system_time_ticks();
    agr->rx[next] = s;
    agr->rx_head = next;
    agr->last_rx_tick = s.rx_tick;
    return true;
}

void agr_handle_health_frame(AGR *agr, const uint8_t *data, uint8_t len) {
    if (len < 7) {
        return;
    }
    agr->node_status = (float) data[0];
    agr->node_gate_pct = (float) data[1];
    agr->node_sat = (float) data[2];
    agr->node_temp_c = (float) data[4] - 64.0f;
    agr->node_cfg_crc = (float) data[6];
}

// process one raw sample through classify -> locate -> profile
static void ingest(AGR *agr, const AgrRawSample *s, const Time *time) {
    bool valid = s->range_mm != AGR_NO_RETURN && s->range_mm != 0;
    // per received slot, never per tick (sight window counts sensor frames)
    agr_trust_sample(&agr->trust, valid);

    uint8_t expected = (uint8_t) (agr->last_counter + 1);
    if (agr->trust.bits_n > 1 && s->counter != expected) {
        agr->can_loss_count += (uint8_t) (s->counter - expected);
    }
    agr->last_counter = s->counter;

    if (!valid) {
        return;  // blindness evidence: routed to trust only, never terrain
    }
    // lag compensation: pitch at emission time = rx age + configured pipeline lag.
    // rx_tick is stamped in the CAN thread while time->now is latched once at the
    // top of the main loop, so a frame landing mid-tick reads newer than now.
    int32_t age_ticks = (int32_t) (time->now - s->rx_tick);
    if (age_ticks < 0) {
        age_ticks = 0;
    }
    float lag = (float) age_ticks * agr->sys_to_main + agr->lag_ticks;
    float pitch = agr_pitch_ring_at(&agr->pitch_ring, lag);
    AgrGroundPoint pt = agr_locate(&agr->geo, pitch, (float) s->range_mm * 0.001f);
    if (!pt.ok) {
        return;
    }
    if (pt.x <= AGR_AHEAD_M) {
        // clearance sweep from the sensor origin toward the hit
        float a = agr->geo.mount_height - agr->geo.wheel_radius;
        float sx = agr->geo.mount_fwd * cosf(pitch) - a * sinf(pitch);
        float sz = agr->geo.wheel_radius + agr->geo.mount_fwd * sinf(pitch) + a * cosf(pitch);
        agr_profile_clear_ray(&agr->profile, sx, sz, pt.x, pt.z);
        // strength scales insertion weight: 0..255 -> 0.25..1.25
        float w = 0.25f + (float) s->strength / 255.0f;
        agr_profile_insert(&agr->profile, pt.x, pt.z, w);
    } else {
        agr_profile_far_hit(&agr->profile, pt.x, pt.z);
    }
    // calibration capture taps the same converted sample
    if (agr->cal_mode == AGR_CAL_SWEEPING) {
        agr_cal_sweep_add(&agr->cal_sweep, pitch, (float) s->range_mm * 0.001f);
    } else if (agr->cal_mode == AGR_CAL_TAU) {
        agr_tau_feed(
            &agr->cal_tau, &agr->pitch_ring, &agr->geo, (float) s->range_mm * 0.001f,
            agr->sys_to_main * (float) SYSTEM_TICK_RATE_HZ
        );
    }
}

void agr_update(
    AGR *agr, const MotorData *motor, const IMU *imu, const Time *time,
    const RefloatConfig *config, float dt
) {
    agr_pitch_ring_push(&agr->pitch_ring, imu->pitch * AGR_DEG2RAD);

    // odometry delta (mc_get_distance is signed net meters)
    float dd = 0.0f;
    if (agr->have_distance) {
        dd = motor->distance - agr->last_distance;
    }
    agr->last_distance = motor->distance;
    agr->have_distance = true;
    if (!isfinite(dd) || fabsf(dd) > 1.0f) {
        dd = 0.0f;  // odometry glitch: a 500 Hz tick can't move 1 m; never feed the profile garbage
    }

    if (agr->cal_mode == AGR_CAL_SWEEPING) {
        // net travel, not erpm: a ground-locked pivot reads erpm equal to pitch
        // rate while the board stays put — the exact motion the sweep asks for
        agr->cal_net_m += dd;
        if (fabsf(agr->cal_net_m) > AGR_CAL_ABORT_NET_M) {
            agr->cal_mode = AGR_CAL_IDLE;
            log_msg("AGR cal: sweep aborted (board moved)");
        }
    }

    if (agr->sim_active) {
        // synthesize one sample per other tick at the ring head (bench tool)
        if ((agr->sim_phase++ & 1) == 0) {
            float pitch = imu->pitch * AGR_DEG2RAD;
            float delta = agr->geo.mount_angle + agr->geo.mount_offset - pitch;
            float a = agr->geo.mount_height - agr->geo.wheel_radius;
            float xs = agr->geo.mount_fwd * cosf(pitch) - a * sinf(pitch);
            float zs = agr->geo.wheel_radius + agr->geo.mount_fwd * sinf(pitch) + a * cosf(pitch);
            float g = agr->sim_grade;
            float denom = sinf(delta) + g * cosf(delta);
            if (denom > 0.01f) {
                float t_ray = (zs - g * xs) / denom;
                AgrRawSample s = {
                    .range_mm = (uint16_t) (t_ray * 1000.0f),
                    .strength = 200,
                    .counter = (uint8_t) (agr->last_counter + 1),
                    .rx_tick = time->now,
                };
                agr->last_rx_tick = time->now;
                ingest(agr, &s, time);
            }
        }
    } else {
        while (agr->rx_tail != agr->rx_head) {
            agr->rx_tail = (uint8_t) ((agr->rx_tail + 1) % AGR_RX_RING);
            AgrRawSample s = agr->rx[agr->rx_tail];  // struct copy from volatile
            ingest(agr, &s, time);
        }
    }

    agr_profile_advance(&agr->profile, dd);
    agr->fit = agr_profile_fit(&agr->profile);

    // fitted grade in degrees, this tick (0 when the fit is not actionable)
    agr->g_cmd = agr->fit.valid ? atanf(agr->fit.slope) * AGR_RAD2DEG : 0.0f;

    int32_t since_rx = (int32_t) (time->now - agr->last_rx_tick);
    bool stale = since_rx > (int32_t) agr->timeout_ticks;
    agr_trust_update(&agr->trust, &agr->fit, stale, dd, &agr->tuning, dt);

    float raw = agr->fit.valid
        ? agr_law(&agr->fit, agr->trust.dir_forward, motor->abs_erpm, &agr->tuning)
        : 0.0f;
    agr_cond_update(&agr->cond, raw, agr->trust.fade, &agr->tuning, dt);

    // live trim (bounded; feeds back through geometry on next configure)
    bool moving = motor->abs_erpm > 500.0f;
    float new_trim =
        agr_trim_update(agr->trim_deg, agr->fit.valid ? agr->g_cmd : 0.0f, agr->trust.fade, moving, dt);
    if (new_trim != agr->trim_deg) {
        agr->trim_deg = new_trim;
        agr->geo.mount_offset = (config->agr_mount_offset + agr->trim_deg) * AGR_DEG2RAD;
    }

    // tau session countdown
    if (agr->cal_mode == AGR_CAL_TAU && agr->cal_tau_ticks > 0 && --agr->cal_tau_ticks == 0) {
        agr->cal_mode = AGR_CAL_TAU_PENDING;
    }

    // telemetry mirrors
    agr->fade = agr->trust.fade;
    agr->gates_f = (float) ((agr->trust.link_ok ? 1 : 0) | (agr->trust.sight_ok ? 2 : 0) |
                            (agr->trust.fit_ok ? 4 : 0) | (agr->trust.policy_ok ? 8 : 0));
    agr->fit_residual = agr->fit.residual;
    agr->fit_weight = agr->fit.weight;
    agr->valid_fraction = agr->trust.valid_fraction;
    agr->far_chord = agr->profile.far_hits >= AGR_FAR_PERSIST
        ? atanf(agr->profile.far_z / agr->profile.far_x) * AGR_RAD2DEG
        : 0.0f;
    agr->law_raw = raw;
    agr->setpoint = agr->cond.setpoint;
}

void agr_winddown(AGR *agr) {
    agr_cond_winddown(&agr->cond);
    agr->setpoint = agr->cond.setpoint;
}
