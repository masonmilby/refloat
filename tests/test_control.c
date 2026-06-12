// forks/refloat/tests/test_control.c
#include "agr_control.h"
#include "agr_math.h"
#include "t.h"

static AgrTuning tun(void) {
    AgrTuning c = {
        .strength_up = 0.3f, .strength_down = 0.2f,
        .angle_limit_up = 8.0f, .angle_limit_down = 4.0f,
        .taper_erpm = 0.0f,
        .sight_on = 0.5f, .sight_off = 0.3f,
        .fade_rate = 2.0f, .reverse_fade_m = 1.0f, .rate_limit = 15.0f,
    };
    return c;
}

int main(void) {
    AgrTuning cfg = tun();
    AgrFit good = {.valid = true, .slope = 0.20f, .residual = 0.005f, .weight = 20.0f, .span = 2.0f};
    float dt = 0.002f;

    // sight hysteresis
    AgrTrust t;
    agr_trust_init(&t);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, true);
    }
    agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(t.sight_ok && t.trusted);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, i % 4 == 0);  // 25% valid: below sight_off
    }
    agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(!t.sight_ok && !t.trusted);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, i % 5 != 0);  // 80% valid: above sight_on
    }
    agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(t.sight_ok);

    // sight warm-up floor: fraction=1.0 but bits_n=10 (<32) must NOT latch
    agr_trust_init(&t);
    for (int i = 0; i < 10; i++) {
        agr_trust_sample(&t, true);
    }
    agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(!t.sight_ok);  // bits_n=10 < AGR_SIGHT_WINDOW/2=32: still cold
    for (int i = 0; i < 30; i++) {
        agr_trust_sample(&t, true);
    }
    agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(t.sight_ok);  // bits_n=40 >= 32 and fraction=1.0: latches

    // fade slews, never steps
    agr_trust_init(&t);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, true);
    }
    agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(t.fade < 0.01f);  // one tick: 2.0/s * 2ms = 0.004
    for (int i = 0; i < 300; i++) {
        agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    }
    CHECK_NEAR(t.fade, 1.0f, 1e-3f);  // 0.6 s at 2.0/s

    // link stale kills trust
    agr_trust_update(&t, &good, true, 1000.0f, 0.01f, &cfg, dt);
    CHECK(!t.trusted);

    // reverse policy: 1.0 m contiguous reverse trips; 0.25 m forward clears
    agr_trust_init(&t);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, true);
    }
    for (int i = 0; i < 99; i++) {
        agr_trust_update(&t, &good, false, -500.0f, -0.01f, &cfg, dt);
    }
    CHECK(t.policy_ok);  // 0.99 m: not yet
    agr_trust_update(&t, &good, false, -500.0f, -0.02f, &cfg, dt);
    CHECK(!t.policy_ok);
    for (int i = 0; i < 26; i++) {
        agr_trust_update(&t, &good, false, 500.0f, 0.01f, &cfg, dt);
    }
    CHECK(t.policy_ok);
    // rocking never trips: ±0.3 m oscillation
    agr_trust_init(&t);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, true);
    }
    for (int cyc = 0; cyc < 10; cyc++) {
        for (int i = 0; i < 30; i++) {
            agr_trust_update(&t, &good, false, -100.0f, -0.01f, &cfg, dt);
        }
        for (int i = 0; i < 30; i++) {
            agr_trust_update(&t, &good, false, 100.0f, 0.01f, &cfg, dt);
        }
    }
    CHECK(t.policy_ok);

    // fit gate hysteresis on residual
    agr_trust_init(&t);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, true);
    }
    AgrFit noisy = good;
    noisy.residual = 0.05f;  // over 0.03 ceiling
    agr_trust_update(&t, &noisy, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(!t.fit_ok);
    noisy.residual = 0.028f;  // under ceiling but over 0.8*ceiling: still off
    agr_trust_update(&t, &noisy, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(!t.fit_ok);
    noisy.residual = 0.02f;  // under recovery threshold
    agr_trust_update(&t, &noisy, false, 1000.0f, 0.01f, &cfg, dt);
    CHECK(t.fit_ok);

    // law: 20% grade = 11.31 deg; uphill forward: 0.3 * 11.31 = 3.39
    float raw = agr_law(&good, true, 1000.0f, &cfg);
    CHECK_NEAR(raw, 0.3f * atanf(0.20f) * AGR_RAD2DEG, 0.01f);
    // downhill (negative slope, forward): strength_down, clamped at -4
    AgrFit steep_down = good;
    steep_down.slope = -0.60f;  // -31 deg * 0.2 = -6.2 -> clamps to -4
    CHECK_NEAR(agr_law(&steep_down, true, 1000.0f, &cfg), -4.0f, 1e-4f);
    // reverse travel flips which strength applies (positive slope = downhill in reverse)
    float raw_rev = agr_law(&good, false, 1000.0f, &cfg);
    CHECK_NEAR(raw_rev, 0.2f * atanf(0.20f) * AGR_RAD2DEG, 0.01f);

    // reverse clamp: tight limit follows travel-leading end
    // dir=reverse, slope=+0.60: uphill=false==true=false -> strength_down=0.2;
    //   0.2 * atan(0.60)deg = 0.2 * 30.964 = 6.193 -> clamped to +4.0 (tight)
    AgrFit slope_pos = good;
    slope_pos.slope = 0.60f;
    CHECK_NEAR(agr_law(&slope_pos, false, 1000.0f, &cfg), 4.0f, 1e-4f);
    // dir=reverse, slope=-0.60: uphill=false==false=true -> strength_up=0.3;
    //   0.3 * atan(-0.60)deg = 0.3 * (-30.964) = -9.289 -> clamped to -8.0 (loose)
    AgrFit slope_neg = good;
    slope_neg.slope = -0.60f;
    CHECK_NEAR(agr_law(&slope_neg, false, 1000.0f, &cfg), -8.0f, 1e-4f);

    // conditioning: order fade -> EMA -> rate limit; rate cap respected
    AgrCond cond;
    agr_cond_init(&cond);
    agr_cond_configure(&cond, 10.0f, 500.0f);
    for (int i = 0; i < 500; i++) {
        agr_cond_update(&cond, 6.0f, 1.0f, &cfg, dt);
    }
    CHECK_NEAR(cond.setpoint, 6.0f, 0.05f);
    agr_cond_init(&cond);
    agr_cond_configure(&cond, 10.0f, 500.0f);
    agr_cond_update(&cond, 100.0f, 1.0f, &cfg, dt);
    CHECK(cond.setpoint <= cfg.rate_limit * dt + 1e-5f);  // rate limit caps the step

    // drift bound: rate limit caps any estimator walk
    // raw_deg steps 0 -> -4 instantly at fade=1; after 50 ticks assert |moved| <= 1.5 + eps
    agr_cond_init(&cond);
    agr_cond_configure(&cond, 10.0f, 500.0f);
    for (int i = 0; i < 50; i++) {
        agr_cond_update(&cond, -4.0f, 1.0f, &cfg, dt);
    }
    CHECK(cond.setpoint >= -(15.0f * (50 * dt) + 0.01f));

    // taper: taper_erpm=1000, abs_erpm=500 -> output halves vs taper-off
    AgrTuning cfg_tap = tun();
    cfg_tap.taper_erpm = 1000.0f;
    float raw_tap = agr_law(&good, true, 500.0f, &cfg_tap);
    float raw_no_tap = agr_law(&good, true, 500.0f, &cfg);
    CHECK_NEAR(raw_tap, raw_no_tap * 0.5f, 0.01f);

    // fade DOWN slew: from fade=1, untrusted ticks decrease fade by fade_rate*dt per tick
    agr_trust_init(&t);
    for (int i = 0; i < 64; i++) {
        agr_trust_sample(&t, true);
    }
    // bring fade up to 1
    for (int i = 0; i < 600; i++) {
        agr_trust_update(&t, &good, false, 1000.0f, 0.01f, &cfg, dt);
    }
    CHECK_NEAR(t.fade, 1.0f, 1e-3f);
    // now go untrusted (stale) and let fade slew down
    float prev_fade = t.fade;
    for (int i = 0; i < 250; i++) {
        agr_trust_update(&t, &good, true, 1000.0f, 0.01f, &cfg, dt);
        CHECK(t.fade <= prev_fade + 1e-6f);  // never increases
        CHECK(t.fade >= 0.0f);               // never negative
        prev_fade = t.fade;
    }
    CHECK_NEAR(t.fade, 0.0f, 1e-3f);  // reaches 0 in ~0.5 s

    // winddown: setpoint 4.0, ema 4.0 -> 200 calls -> ~4 * 0.995^200 ~= 1.472
    agr_cond_init(&cond);
    cond.setpoint = 4.0f;
    cond.ema.value = 4.0f;
    for (int i = 0; i < 200; i++) {
        agr_cond_winddown(&cond);
    }
    CHECK_NEAR(cond.setpoint, 1.472f, 0.05f);

    T_REPORT();
}
