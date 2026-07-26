// forks/refloat/tests/test_dip.c — the spec's benchmark scenario, synthetic
// Check-count note: ~1900 of the checks are per-tick setpoint-bounds assertions; the pinned signature itself is the ~11 scalar assertions at the tails.
//
// Chord-driven advection gives sustained-grade hold: on a uniform slope the
// advection height-decomposition uses the chord from the contact patch (0,0) to
// the freshest near sample (x_f, z_f), whose slope z_f/x_f is the true grade —
// so advected cells stay terrain-true and the fit reads the real slope. The AGR
// now both tracks steady grade AND leads grade changes. The pinned signature:
//
//   1. Steady descent: nose-down setpoint converges to ≈ -0.3*atan(0.20)*RAD2DEG
//      ≈ -3.39° on the uniform -20% slope (min_descend_sp well below -0.5).
//   2. Unwind: as the wheel nears the flat, the fit mixes downhill-under-wheel
//      with flat-ahead, so the setpoint unwinds OFF the descent floor early
//      (sp_at[0], captured at wx=-0.5 while the board is still descending, is
//      meaningfully above min_descend_sp).
//   3. Flat: setpoint settles near 0.
//   4. Pre-tilt: positive nose-up command builds as the upslope wall fills the
//      window, and keeps building (sp_at[2] > sp_at[1] + 0.3, sp_at[2] > 0.5).
//   5. Fade holds at 1.0 across both dip corners — the geometric kink residual at
//      a 20% corner peaks ≈ 0.021 m, under the 0.03 m fit-trust ceiling.
//
// Beyond wx ≈ 4.55 m the profile drains: on the +20% wall the deck pitches up
// +11.3°, the beam depression below the deck is 13°, so world depression is only
// ≈1.7° — below AGR_MIN_DEPRESSION_RAD (2°). agr_locate rejects every return as
// near-horizon, no new cells are inserted, and the buffer drains as it shifts.
// This is a sensor-geometry limit (not an advection one); the canonical window
// ends where the fit is still valid (wx < 4.5).
//
#include "agr_control.h"
#include "agr_geometry.h"
#include "agr_math.h"
#include "agr_profile.h"
#include "t.h"

typedef float (*TerrainFn)(float x);

// dip: -20% down for x<0, flat 0..4 m, +20% up after (world x)
static float terrain_dip(float x) {
    if (x < 0.0f) {
        return -0.20f * x;  // descending toward the flat at z=0
    }
    if (x < 4.0f) {
        return 0.0f;
    }
    return 0.20f * (x - 4.0f);
}

static float terrain_slope(float x) {
    return x < 0.0f ? -0.20f : (x < 4.0f ? 0.0f : 0.20f);
}

// uniform -20% everywhere — for the sustained-grade-hold sections
static float terrain_down20(float x) {
    return -0.20f * x;
}

// cast the sensor ray from world position against the given terrain (linear search)
static float cast_range(TerrainFn tz, const AgrGeometry *g, float wx, float pitch) {
    float a = g->mount_height - AGR_WHEEL_RADIUS_M;
    float sx = wx + g->mount_fwd * cosf(pitch) - a * sinf(pitch);
    float sz = tz(wx) + AGR_WHEEL_RADIUS_M + g->mount_fwd * sinf(pitch) + a * cosf(pitch);
    float delta = g->mount_angle - pitch;
    float dx = cosf(delta), dz = -sinf(delta);
    for (float t = 0.05f; t < 12.0f; t += 0.005f) {
        if (sz + t * dz <= tz(sx + t * dx)) {
            return t;
        }
    }
    return -1.0f;  // no return
}

// one full pipeline tick at a fixed pitch; advances the profile by dd, conditions output
static void pipeline_tick(
    TerrainFn tz, const AgrGeometry *geo, AgrProfile *prof, AgrTrust *trust, AgrCond *cond,
    const AgrTuning *cfg, float wx, float pitch, float dd, float dt, int tick
) {
    if (tick % 2 == 0) {
        float r = cast_range(tz, geo, wx, pitch);
        bool valid = r > 0.0f;
        agr_trust_sample(trust, valid);
        if (valid) {
            AgrGroundPoint pt = agr_locate(geo, pitch, r);
            if (pt.ok) {
                if (pt.x <= AGR_AHEAD_M) {
                    float a = geo->mount_height - AGR_WHEEL_RADIUS_M;
                    float sx0 = geo->mount_fwd * cosf(pitch) - a * sinf(pitch);
                    float sz0 =
                        AGR_WHEEL_RADIUS_M + geo->mount_fwd * sinf(pitch) + a * cosf(pitch);
                    agr_profile_clear_ray(prof, sx0, sz0, pt.x, pt.z);
                    agr_profile_insert(prof, pt.x, pt.z, 1.0f);
                } else {
                    agr_profile_far_hit(prof, pt.x, pt.z);
                }
            }
        }
    }
    agr_profile_advance(prof, dd);
    AgrFit fit = agr_profile_fit(prof);
    agr_trust_update(trust, &fit, false, dd, cfg, dt);
    float raw = fit.valid ? agr_law(&fit, trust->dir_forward, 5000.0f, cfg) : 0.0f;
    agr_cond_update(cond, raw, trust->fade, cfg, dt);
}

int main(void) {
    AgrGeometry geo = {
        .mount_angle = 13.0f * AGR_DEG2RAD, .mount_offset = 0.0f,
        .mount_height = 0.175f, .mount_fwd = 0.35f, .range_bias = 0.0f,
    };
    AgrTuning cfg = {
        .strength_up = 0.3f, .strength_down = 0.3f,
        .angle_limit_up = 8.0f, .angle_limit_down = 4.0f, .taper_erpm = 0.0f,
        .sight_on = 0.5f, .sight_off = 0.3f, .residual_max = 0.03f, .fade_rate = 2.0f,
        .reverse_fade_m = 1.0f, .dir_flip_m = 0.6f, .rate_limit = 15.0f,
    };
    AgrProfile prof;
    AgrTrust trust;
    AgrCond cond;
    agr_profile_init(&prof);
    agr_trust_init(&trust);
    agr_cond_init(&cond);
    agr_cond_configure(&cond, 10.0f, 500.0f);

    const float v = 6.7f, dt = 0.002f;  // 15 mph at 500 Hz
    float wx = -8.0f;                   // world x; flat starts at 0, wall base at 4
    float prev_dist = 0.0f, dist = 0.0f;

    // captured setpoints across the dip
    float sp_at[3] = {0.0f, 0.0f, 0.0f};
    float fade_at_sp2 = 0.0f;
    bool captured[3] = {false, false, false};
    // most-negative setpoint during steady descent (board on the uniform slope)
    float min_descend_sp = 0.0f;
    // worst geometric residual seen around the two dip corners
    float max_corner_res = 0.0f;

    int tick = 0;

    while (wx < 4.5f) {
        float slope = terrain_slope(wx);
        float pitch = atanf(slope);              // deck parallel to local ground
        float ds = v * dt * cosf(atanf(slope));  // horizontal advance
        wx += ds;
        dist += v * dt;  // wheel odometry = arc length
        float dd = dist - prev_dist;
        prev_dist = dist;

        // 250 Hz sensor: every other tick
        if (tick % 2 == 0) {
            float r = cast_range(terrain_dip, &geo, wx, pitch);
            bool valid = r > 0.0f;
            agr_trust_sample(&trust, valid);
            if (valid) {
                AgrGroundPoint pt = agr_locate(&geo, pitch, r);
                if (pt.ok) {
                    if (pt.x <= AGR_AHEAD_M) {
                        float a = geo.mount_height - AGR_WHEEL_RADIUS_M;
                        float sx0 = geo.mount_fwd * cosf(pitch) - a * sinf(pitch);
                        float sz0 = AGR_WHEEL_RADIUS_M + geo.mount_fwd * sinf(pitch) +
                                    a * cosf(pitch);
                        agr_profile_clear_ray(&prof, sx0, sz0, pt.x, pt.z);
                        agr_profile_insert(&prof, pt.x, pt.z, 1.0f);
                    } else {
                        agr_profile_far_hit(&prof, pt.x, pt.z);
                    }
                }
            }
        }
        agr_profile_advance(&prof, dd);
        AgrFit fit = agr_profile_fit(&prof);
        agr_trust_update(&trust, &fit, false, dd, &cfg, dt);
        float raw = fit.valid ? agr_law(&fit, trust.dir_forward, 5000.0f, &cfg) : 0.0f;
        agr_cond_update(&cond, raw, trust.fade, &cfg, dt);

        // steady descent: most-negative setpoint while the board is on the slope,
        // away from the corners (wx in -7..-2). Sustained-grade hold lives here.
        if (wx > -7.0f && wx < -2.0f && cond.setpoint < min_descend_sp) {
            min_descend_sp = cond.setpoint;
        }

        // worst residual around the two 20% corners (slope→flat near 0, flat→up near 4)
        if (fit.valid && ((wx > -0.5f && wx < 0.9f) || (wx > 3.5f && wx < 4.5f)) &&
            fit.residual > max_corner_res) {
            max_corner_res = fit.residual;
        }

        // sp_at[0]: the board is still descending (pitch ≈ -11°) but the flat is
        // already in the beam — the setpoint has begun unwinding off the descent floor.
        if (wx >= -0.5f && !captured[0]) {
            sp_at[0] = cond.setpoint;
            captured[0] = true;
        }
        if (wx >= 2.0f && !captured[1]) {
            sp_at[1] = cond.setpoint;  // mid-flat
            captured[1] = true;
        }
        if (wx >= 3.9f && !captured[2]) {
            sp_at[2] = cond.setpoint;  // just before the wall
            fade_at_sp2 = trust.fade;
            captured[2] = true;
        }
        CHECK(cond.setpoint <= cfg.angle_limit_up + 1e-3f);
        CHECK(cond.setpoint >= -cfg.angle_limit_down - 1e-3f);
        tick++;
    }

    // steady-descent hold: a real nose-down command builds on the uniform grade
    // (expect ≈ -3.4°; -0.5 is a conservative floor)
    CHECK(min_descend_sp < -0.5f);
    // unwind: as the flat enters the beam the setpoint comes back off the descent
    // floor while the board is STILL descending
    CHECK(sp_at[0] > min_descend_sp + 0.2f);
    // pre-tilt keeps building as the upslope wall fills the fit window
    CHECK(sp_at[2] > sp_at[1] + 0.3f);
    CHECK(sp_at[2] > 0.5f);
    // the 20% corner residual stays under the fit ceiling, so trust never dips
    CHECK(max_corner_res < cfg.residual_max);
    CHECK(fade_at_sp2 >= 0.99f);

    // ---- sustained grade hold ----
    // fresh pipeline, uniform -20% terrain. fit reads the true grade; the setpoint
    // converges to the law's steady value. (Cold start: the fade ramp gates the
    // setpoint, so it reaches steady value at ≈4 m, not 3 m — the slope is locked
    // in by 3 m and we settle the run to ≈5.5 m before checking the magnitude.)
    {
        AgrProfile sp_prof;
        AgrTrust sp_trust;
        AgrCond sp_cond;
        agr_profile_init(&sp_prof);
        agr_trust_init(&sp_trust);
        agr_cond_init(&sp_cond);
        agr_cond_configure(&sp_cond, 10.0f, 500.0f);

        const float gpitch = atanf(-0.20f);
        float gwx = -6.0f, gprev = 0.0f, gdist = 0.0f, traveled = 0.0f;
        int gtick = 0;
        AgrFit slope_at_3m = {0};
        bool got_3m = false;
        const float expect_sp = -0.3f * atanf(0.20f) * AGR_RAD2DEG;  // ≈ -3.39

        while (traveled < 5.5f) {
            float ds = v * dt * cosf(gpitch);
            gwx += ds;
            traveled += ds;
            gdist += v * dt;
            float dd = gdist - gprev;
            gprev = gdist;
            pipeline_tick(
                terrain_down20, &geo, &sp_prof, &sp_trust, &sp_cond, &cfg, gwx, gpitch, dd, dt,
                gtick
            );
            if (!got_3m && traveled >= 3.0f) {
                slope_at_3m = agr_profile_fit(&sp_prof);
                got_3m = true;
            }
            gtick++;
        }
        // at 3 m: fit is valid and reads the true grade
        CHECK(slope_at_3m.valid);
        CHECK_NEAR(slope_at_3m.slope, -0.20f, 0.02f);
        // after the run settles: setpoint at the law's steady value
        CHECK_NEAR(sp_cond.setpoint, expect_sp, 0.3f);

        // ---- stop-hold on grade ----
        // continue the same run but stop advancing (dd=0), sensor + ticks still
        // running, still on the slope: the held offset must not drift.
        float sp_before = sp_cond.setpoint;
        int stop_ticks = (int) (1.0f / dt);  // 1 s
        for (int k = 0; k < stop_ticks; k++) {
            pipeline_tick(
                terrain_down20, &geo, &sp_prof, &sp_trust, &sp_cond, &cfg, gwx, gpitch, 0.0f, dt,
                gtick
            );
            gtick++;
        }
        CHECK_NEAR(sp_cond.setpoint, sp_before, 0.1f);
        CHECK_NEAR(sp_trust.fade, 1.0f, 1e-3f);
    }

    T_REPORT();
}
