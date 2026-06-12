// forks/refloat/tests/test_cal.c
#include "agr_cal_core.h"
#include "agr_geometry.h"
#include "agr_math.h"
#include "t.h"

#include <stdlib.h>

static float noise(unsigned *seed) {  // deterministic ±5 mm uniform
    *seed = *seed * 1103515245u + 12345u;
    return (((*seed >> 16) & 0x7fff) / 32767.0f - 0.5f) * 0.010f;
}

int main(void) {
    const float cad = 13.0f * AGR_DEG2RAD;
    const float h = 0.18f, f = 0.36f, te = 0.8f * AGR_DEG2RAD, b = 0.02f;
    AgrCalSweep s;
    agr_cal_sweep_init(&s);
    unsigned seed = 42;
    // sweep pitch -20..+4 deg (delta 33..9 deg), 12 samples per 0.25 deg
    for (float pd = -20.0f; pd <= 4.0f; pd += 0.25f) {
        float p = pd * AGR_DEG2RAD;
        for (int k = 0; k < 12; k++) {
            float r = agr_cal_model(p, cad, te, h, f, b) + noise(&seed);
            agr_cal_sweep_add(&s, p, r);
        }
    }
    AgrCalResult res = agr_cal_fit(&s, cad);
    CHECK(res.status == AGR_CAL_OK);
    CHECK_NEAR(res.mount_height, h, 0.005f);
    CHECK_NEAR(res.mount_fwd, f, 0.015f);
    // θe/b are jointly ill-conditioned over a flat-ground sweep (JtJ condition ~1e9);
    // double accumulation + light damping handle it; individual recovery is seed-dependent
    // but locate-functional accuracy is bounded (see functional-equivalence test below).
    CHECK_NEAR(res.mount_offset_deg, 0.8f, 0.30f);
    CHECK_NEAR(res.range_bias, b, 0.020f);
    CHECK(res.rms_m < 0.008f);

    // ---- functional equivalence: recovered params match truth through agr_locate ----
    // θe/b are individually ill-conditioned (condition ~1e9 over this sweep) but are
    // only ever consumed jointly via agr_locate; the contract is locate accuracy.
    {
        AgrGeometry g_true = {.mount_angle = cad, .mount_offset = te,
                              .mount_height = h, .mount_fwd = f, .range_bias = b};
        AgrGeometry g_fit = {.mount_angle = cad, .mount_offset = res.mount_offset_deg * AGR_DEG2RAD,
                             .mount_height = res.mount_height, .mount_fwd = res.mount_fwd,
                             .range_bias = res.range_bias};
        float max_dz = 0.0f;
        for (float pd = -6.0f; pd <= 6.0f; pd += 0.5f) {
            float p = pd * AGR_DEG2RAD;
            float r_true = agr_cal_model(p, cad, te, h, f, b);  // flat-ground range, truth
            AgrGroundPoint a_true = agr_locate(&g_true, p, r_true);
            AgrGroundPoint a_fit = agr_locate(&g_fit, p, r_true);
            if (a_true.ok && a_fit.ok) {
                float dz = fabsf(a_fit.z - a_true.z);
                if (dz > max_dz) max_dz = dz;
            }
        }
        CHECK(max_dz < 0.006f);  // reviewer-measured worst: 5.6 mm at +6° (seed 42)
    }

    // partial sweep refuses
    AgrCalSweep tiny;
    agr_cal_sweep_init(&tiny);
    for (float pd = -2.0f; pd <= 2.0f; pd += 0.25f) {
        float p = pd * AGR_DEG2RAD;
        for (int k = 0; k < 12; k++) {
            agr_cal_sweep_add(&tiny, p, agr_cal_model(p, cad, 0.0f, h, f, b));
        }
    }
    CHECK(agr_cal_fit(&tiny, cad).status == AGR_CAL_COVERAGE);

    // --- added tests ---

    // bin indexing: verify the (pd + 40) / 0.5 formula maps correctly.
    // pitch -40.0 -> bin 0; pitch +39.99 -> bin 159; pitch +40.1 -> ignored; range<=0 ignored.
    // Note: at pitch=-40° the model returns a negative range (sensor below ground), so we
    // use a valid positive range literal of 1.0 m directly for the bin-index test.
    {
        AgrCalSweep bi;
        agr_cal_sweep_init(&bi);
        // bin 0: pitch -40° -> i = (-40+40)/0.5 = 0
        float p_neg40 = -40.0f * AGR_DEG2RAD;
        for (int k = 0; k < 6; k++) {
            agr_cal_sweep_add(&bi, p_neg40, 1.0f);  // valid positive range
        }
        CHECK(bi.bin[0].n == 6);
        // bin 159: pitch +39.99° -> i = (39.99+40)/0.5 = 159 (floor)
        float p_pos3999 = 39.99f * AGR_DEG2RAD;
        for (int k = 0; k < 6; k++) {
            agr_cal_sweep_add(&bi, p_pos3999, 1.0f);
        }
        CHECK(bi.bin[159].n == 6);
        // out-of-range pitch +40.1° -> i=160 -> ignored
        float p_out = 40.1f * AGR_DEG2RAD;
        agr_cal_sweep_add(&bi, p_out, 1.0f);
        CHECK(bi.bin[159].n == 6);  // unchanged
        // range <= 0 ignored: bin for -10° = (-10+40)/0.5 = 60
        agr_cal_sweep_add(&bi, -10.0f * AGR_DEG2RAD, 0.0f);
        agr_cal_sweep_add(&bi, -10.0f * AGR_DEG2RAD, -1.0f);
        CHECK(bi.bin[60].n == 0);
    }

    // n<5 exclusion: a sufficient sweep plus one bin with n=4 at a grossly wrong range.
    // The fit result must be unchanged (within 1e-4) since that bin is excluded.
    {
        // Build reference sweep: same as the canonical test, deterministic
        AgrCalSweep ref;
        agr_cal_sweep_init(&ref);
        unsigned seed2 = 42;
        for (float pd = -20.0f; pd <= 4.0f; pd += 0.25f) {
            float p = pd * AGR_DEG2RAD;
            for (int k = 0; k < 12; k++) {
                float r = agr_cal_model(p, cad, te, h, f, b) + noise(&seed2);
                agr_cal_sweep_add(&ref, p, r);
            }
        }
        AgrCalResult ref_res = agr_cal_fit(&ref, cad);
        CHECK(ref_res.status == AGR_CAL_OK);

        // Build a copy with 4 extra samples at -5° (bin = (-5+40)/0.5 = 70) at wrong range.
        // delta at -5° = 13 - (-5) = 18° > 8°, so this bin passes the delta gate.
        // But n=4 < AGR_CAL_MIN_N=5 so it gets excluded.
        AgrCalSweep with_bad;
        agr_cal_sweep_init(&with_bad);
        unsigned seed3 = 42;
        for (float pd = -20.0f; pd <= 4.0f; pd += 0.25f) {
            float p = pd * AGR_DEG2RAD;
            for (int k = 0; k < 12; k++) {
                float r = agr_cal_model(p, cad, te, h, f, b) + noise(&seed3);
                agr_cal_sweep_add(&with_bad, p, r);
            }
        }
        // Add 4 samples (n<5) at -22° (outside the -20..+4 sweep, delta=35°>8°), wrong range.
        // n=4 < AGR_CAL_MIN_N=5 so this bin is excluded from the fit.
        float bad_pitch = -22.0f * AGR_DEG2RAD;
        for (int k = 0; k < 4; k++) {
            agr_cal_sweep_add(&with_bad, bad_pitch, 99.0f);  // 99 m: impossible range
        }
        int bad_bin = (int) ((-22.0f + 40.0f) / AGR_CAL_BIN_DEG);  // = 36
        CHECK(with_bad.bin[bad_bin].n == 4);

        AgrCalResult bad_res = agr_cal_fit(&with_bad, cad);
        CHECK(bad_res.status == AGR_CAL_OK);
        // Results must agree: the n=4 bin was excluded
        CHECK_NEAR(bad_res.mount_height, ref_res.mount_height, 1e-4f);
        CHECK_NEAR(bad_res.mount_fwd, ref_res.mount_fwd, 1e-4f);
        CHECK_NEAR(bad_res.mount_offset_deg, ref_res.mount_offset_deg, 1e-4f);
        CHECK_NEAR(bad_res.range_bias, ref_res.range_bias, 1e-4f);
    }

    // SINGULAR path analysis:
    // The coverage gate requires used >= 10 AND span_deg >= 15°. A set of 10+ bins
    // spanning 15°+ of pitch always provides numerically independent Jacobian columns:
    // - dR/dh depends on pitch through cos(pitch)
    // - dR/df depends on pitch through sin(pitch)
    // - dR/dtheta_err depends on pitch through -cos(delta)/sin^2(delta)
    // - dR/db = 1 (constant)
    // With 10+ distinct pitch values spanning 15°, the trig terms vary enough that
    // the 4x4 normal matrix JtJ cannot be singular. The coverage gate therefore
    // provably precedes any singular configuration in practice.
    // We verify the single-pitch coverage refusal to pin the gate ordering:
    {
        AgrCalSweep single;
        agr_cal_sweep_init(&single);
        float p = -10.0f * AGR_DEG2RAD;  // one pitch, many samples
        for (int k = 0; k < 100; k++) {
            agr_cal_sweep_add(&single, p, agr_cal_model(p, cad, 0.0f, h, f, b));
        }
        // span_deg = 0 for a single bin -> coverage refusal
        AgrCalResult sr = agr_cal_fit(&single, cad);
        CHECK(sr.status == AGR_CAL_COVERAGE);
    }

    // model self-consistency with agr_locate:
    // agr_cal_model(pitch, cad, te, h, f, b) and agr_locate must agree:
    // feeding the model's output range back into locate (with matching geometry)
    // must yield z ≈ 0 (on flat ground) within 2 mm.
    // Reasoning: agr_locate computes r_internal = range - geo.range_bias
    //   = (zs/sin(delta) + b) - b = zs/sin(delta)
    // then z = zs - r_internal * sin(delta) = zs - zs = 0. Exact by construction.
    {
        float pitch = -10.0f * AGR_DEG2RAD;
        float r = agr_cal_model(pitch, cad, te, h, f, b);

        // agr_locate uses mount_angle + mount_offset as total depression.
        // cad + te is the total angle; set mount_offset=te so the sum is correct.
        AgrGeometry geo = {
            .mount_angle = cad,
            .mount_offset = te,  // theta_err as offset
            .mount_height = h,
            .mount_fwd = f,
            .range_bias = b,
        };
        AgrGroundPoint pt = agr_locate(&geo, pitch, r);
        CHECK(pt.ok);
        CHECK_NEAR(pt.z, 0.0f, 0.002f);
    }

    // --- tau scan: synthetic rocking with 20 ms true lag ---
    {
        AgrGeometry geo = {.mount_angle = cad, .mount_offset = 0.0f, .mount_height = h,
                           .mount_fwd = f, .range_bias = b};
        AgrPitchRing ring;
        agr_pitch_ring_init(&ring);
        AgrTauScan tau;
        agr_tau_init(&tau);
        const float tick_hz = 500.0f;
        float t_now = 0.0f;
        float pitch_hist[64] = {0};  // enough for 20 ms at 500 Hz
        int ph = 0;
        for (int i = 0; i < 5000; i++) {  // 10 s of rocking at 1.2 Hz, +-8 deg
            t_now += 1.0f / tick_hz;
            float pitch = 8.0f * AGR_DEG2RAD * sinf(2.0f * 3.14159265f * 1.2f * t_now);
            agr_pitch_ring_push(&ring, pitch);
            pitch_hist[ph % 64] = pitch;
            ph++;
            if (i % 2 == 0 && ph > 10) {  // 250 Hz sensor, range lags pitch by 10 ticks
                float lagged = pitch_hist[(ph - 1 - 10) % 64];
                float r = agr_cal_model(lagged, cad, 0.0f, h, f, b);
                agr_tau_feed(&tau, &ring, &geo, r, tick_hz);
            }
        }
        float tau_ms = agr_tau_result(&tau);
        CHECK(tau_ms > 18.0f && tau_ms < 22.0f);
    }

    // --- live trim: converges against a planted offset error, clamps, freezes ---
    // Model: g_cmd = planted_bias - trim (trim corrects subtractively through locate).
    // Equilibrium: g_cmd = 0 -> trim = planted_bias = -0.3 deg.
    // TAU=600 s; run 5*TAU so trim reaches >99% of target (within 0.01 deg of -0.3).
    {
        float trim = 0.0f;
        for (int i = 0; i < 500 * 3000; i++) {  // 50 min = 5 tau
            trim = agr_trim_update(trim, -0.3f - trim, 1.0f, true, 0.002f);
        }
        CHECK_NEAR(trim, -0.3f, 0.05f);
        float frozen = agr_trim_update(trim, 5.0f, 0.5f, true, 0.002f);
        CHECK_NEAR(frozen, trim, 1e-6f);
        for (int i = 0; i < 500 * 3600; i++) {
            trim = agr_trim_update(trim, -5.0f, 1.0f, true, 0.002f);
        }
        CHECK_NEAR(trim, -AGR_TRIM_LIMIT_DEG, 1e-3f);
    }

    // --- tau: insufficient samples (n=400) returns -1 ---
    {
        AgrGeometry geo2 = {.mount_angle = cad, .mount_offset = 0.0f, .mount_height = h,
                            .mount_fwd = f, .range_bias = b};
        AgrPitchRing ring2;
        agr_pitch_ring_init(&ring2);
        AgrTauScan tau2;
        agr_tau_init(&tau2);
        for (int i = 0; i < 400; i++) {
            float pitch = 8.0f * AGR_DEG2RAD * sinf(2.0f * 3.14159265f * 1.2f * (float) i / 500.0f);
            agr_pitch_ring_push(&ring2, pitch);
            float r = agr_cal_model(pitch, cad, 0.0f, h, f, b);
            agr_tau_feed(&tau2, &ring2, &geo2, r, 500.0f);
        }
        CHECK(tau2.n == 400);
        CHECK_NEAR(agr_tau_result(&tau2), -1.0f, 1e-6f);
    }

    // --- tau at 0 ms lag: true lag 0 ticks -> result < 2 ms ---
    {
        AgrGeometry geo3 = {.mount_angle = cad, .mount_offset = 0.0f, .mount_height = h,
                            .mount_fwd = f, .range_bias = b};
        AgrPitchRing ring3;
        agr_pitch_ring_init(&ring3);
        AgrTauScan tau3;
        agr_tau_init(&tau3);
        float t3 = 0.0f;
        for (int i = 0; i < 5000; i++) {
            t3 += 1.0f / 500.0f;
            float pitch = 8.0f * AGR_DEG2RAD * sinf(2.0f * 3.14159265f * 1.2f * t3);
            agr_pitch_ring_push(&ring3, pitch);
            if (i % 2 == 0) {
                // range uses CURRENT pitch (0 ms lag)
                float r = agr_cal_model(pitch, cad, 0.0f, h, f, b);
                agr_tau_feed(&tau3, &ring3, &geo3, r, 500.0f);
            }
        }
        float tau3_ms = agr_tau_result(&tau3);
        CHECK(tau3_ms < 2.0f);
    }

    // --- tau at 60 ms lag (30 ticks): edge-candidate behavior ---
    // At lag 60 ms = candidate index 15 (the last one, AGR_TAU_CANDIDATES-1),
    // parabolic interpolation is skipped (no right neighbor). The result is
    // exactly candidate 15's ms value = 60.0 ms (no interpolation at edge).
    {
        AgrGeometry geo4 = {.mount_angle = cad, .mount_offset = 0.0f, .mount_height = h,
                            .mount_fwd = f, .range_bias = b};
        AgrPitchRing ring4;
        agr_pitch_ring_init(&ring4);
        AgrTauScan tau4;
        agr_tau_init(&tau4);
        float t4 = 0.0f;
        float pitch_hist4[64] = {0};
        int ph4 = 0;
        for (int i = 0; i < 5000; i++) {
            t4 += 1.0f / 500.0f;
            float pitch = 8.0f * AGR_DEG2RAD * sinf(2.0f * 3.14159265f * 1.2f * t4);
            agr_pitch_ring_push(&ring4, pitch);
            pitch_hist4[ph4 % 64] = pitch;
            ph4++;
            if (i % 2 == 0 && ph4 > 30) {  // range lags by 30 ticks = 60 ms
                float lagged = pitch_hist4[(ph4 - 1 - 30) % 64];
                float r = agr_cal_model(lagged, cad, 0.0f, h, f, b);
                agr_tau_feed(&tau4, &ring4, &geo4, r, 500.0f);
            }
        }
        float tau4_ms = agr_tau_result(&tau4);
        // Winner is candidate 15 (60 ms); edge case skips parabolic interp -> returns 60.0
        CHECK_NEAR(tau4_ms, 60.0f, 1e-3f);
    }

    // --- trim freeze when not moving ---
    {
        float trim_frozen = 0.1f;
        float result = agr_trim_update(trim_frozen, 5.0f, 1.0f, false, 0.002f);
        CHECK_NEAR(result, trim_frozen, 1e-6f);
    }

    // --- trim sign pin: positive g_cmd bias converges to +0.3 ---
    // Model: g_cmd = planted_bias - trim; equilibrium trim = +0.3 for bias = +0.3.
    {
        float trim = 0.0f;
        for (int i = 0; i < 500 * 3000; i++) {
            trim = agr_trim_update(trim, 0.3f - trim, 1.0f, true, 0.002f);
        }
        CHECK_NEAR(trim, 0.3f, 0.05f);
    }

    T_REPORT();
}
