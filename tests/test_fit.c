// forks/refloat/tests/test_fit.c
#include "agr_profile.h"
#include "t.h"

int main(void) {
    // ---- clean 20% ramp ----
    // float loop x in [0.30, 2.00] step AGR_CELL_M → i=11..44
    // agr_cell_x(11)=0.325, agr_cell_x(44)=1.975
    AgrProfile p;
    agr_profile_init(&p);
    for (int i = 11; i <= 44; i++) {
        float x = agr_cell_x(i);
        agr_profile_insert(&p, x, 0.20f * x, 4.0f);
    }
    AgrFit f = agr_profile_fit(&p);
    CHECK(f.valid);
    CHECK_NEAR(f.slope, 0.20f, 0.01f);
    CHECK(f.residual < 0.01f);

    // ---- starved: empty profile → valid=false ----
    AgrProfile e;
    agr_profile_init(&e);
    CHECK(!agr_profile_fit(&e).valid);
    // one cell: span floor fails (span = 0)
    agr_profile_insert(&e, 0.5f, 0.0f, 1.0f);
    CHECK(!agr_profile_fit(&e).valid);

    // ---- far-hit persistence ----
    // float loop x in [0.05, 0.80] step AGR_CELL_M → i=6..20 (centers 0.075..0.775)
    AgrProfile c;
    agr_profile_init(&c);
    for (int i = 6; i <= 20; i++) {
        agr_profile_insert(&c, agr_cell_x(i), 0.0f, 4.0f);
    }
    AgrFit flat = agr_profile_fit(&c);
    CHECK(flat.valid);
    CHECK_NEAR(flat.slope, 0.0f, 0.01f);

    // one hit: not yet trusted
    agr_profile_far_hit(&c, 5.7f, -1.13f);
    AgrFit one = agr_profile_fit(&c);
    CHECK_NEAR(one.slope, flat.slope, 0.01f);

    // 4 more consistent hits → 5 total: trusted
    for (int i = 0; i < 4; i++) {
        agr_profile_far_hit(&c, 5.7f, -1.13f);
    }
    AgrFit crest = agr_profile_fit(&c);
    CHECK(crest.valid);
    CHECK(crest.slope < -0.15f && crest.slope > -0.22f);

    // ---- jump resets persistence ----
    // z jumped from -1.13 to -0.10: |Δz| = 1.03 > AGR_FAR_DZ=0.3 → restart → far_hits=1
    agr_profile_far_hit(&c, 5.7f, -0.10f);
    CHECK(c.far_hits == 1);

    // ---- beyond 6 m ignored ----
    agr_profile_far_reset(&c);
    for (int i = 0; i < 6; i++) {
        agr_profile_far_hit(&c, 7.0f, -1.4f);
    }
    CHECK(c.far_hits == 0);

    // ---- residual reflects scatter ----
    // 20% ramp + alternating ±0.02 m z-noise (even index adds, odd subtracts)
    // Expected residual in (0.015, 0.03) based on ±0.02 perturbation over many points
    AgrProfile sc;
    agr_profile_init(&sc);
    for (int i = 11; i <= 44; i++) {
        float x = agr_cell_x(i);
        float noise = (i % 2 == 0) ? 0.02f : -0.02f;
        agr_profile_insert(&sc, x, 0.20f * x + noise, 4.0f);
    }
    AgrFit sf = agr_profile_fit(&sc);
    CHECK(sf.valid);
    CHECK_NEAR(sf.slope, 0.20f, 0.02f);
    CHECK(sf.residual > 0.015f && sf.residual < 0.03f);

    // ---- boundary: x=6.0 counted (> AGR_FAR_MAX_M is false), x=6.01 resets ----
    AgrProfile b6;
    agr_profile_init(&b6);
    agr_profile_far_hit(&b6, 6.0f, -1.0f);
    CHECK(b6.far_hits == 1);  // boundary: x=6.0 is NOT > 6.0, so counted
    agr_profile_far_reset(&b6);
    for (int i = 0; i < 6; i++) {
        agr_profile_far_hit(&b6, 6.01f, -1.0f);
    }
    CHECK(b6.far_hits == 0);  // x=6.01 > 6.0 → resets every call

    // ---- horizon: x=2.4 ≤ AGR_AHEAD_M=2.5 → resets ----
    AgrProfile bh;
    agr_profile_init(&bh);
    for (int i = 0; i < 6; i++) {
        agr_profile_far_hit(&bh, 2.4f, 0.0f);
    }
    CHECK(bh.far_hits == 0);  // x=2.4 <= 2.5 → reset every call

    // ---- hand-anchored slope: 8 cells (i=6..13) at z=0, w=4 + far (5.7,-1.13) w=4 ----
    // (8 cells, not 5: near span must clear AGR_FIT_MIN_SPAN=0.30 on its own — the
    //  far point no longer stretches the span. centers 0.075..0.425 → near span 0.35.)
    // sums (cells contribute 0 to sz/sxz since z=0):
    //   sw  = 8*4 + 4 = 36.0
    //   sx  = 4*(0.075+0.125+0.175+0.225+0.275+0.325+0.375+0.425) + 4*5.7
    //       = 4*2.0 + 22.8 = 8.0 + 22.8 = 30.8
    //   sz  = 0 + 4*(-1.13) = -4.52
    //   sxx = 4*(0.075^2+...+0.425^2) + 4*5.7^2 = 4*0.605 + 4*32.49 = 2.42 + 129.96 = 132.38
    //   sxz = 0 + 4*5.7*(-1.13) = -25.764
    //   det = 132.38 - 30.8^2/36 = 132.38 - 948.64/36 = 132.38 - 26.35111 = 106.02889
    //   slope = (-25.764 - 30.8*(-4.52)/36) / 106.02889
    //         = (-25.764 + 3.86711) / 106.02889 = -21.89689 / 106.02889 ≈ -0.20652
    AgrProfile ha;
    agr_profile_init(&ha);
    for (int i = 6; i <= 13; i++) {
        agr_profile_insert(&ha, agr_cell_x(i), 0.0f, 4.0f);
    }
    for (int i = 0; i < AGR_FAR_PERSIST; i++) {
        agr_profile_far_hit(&ha, 5.7f, -1.13f);
    }
    AgrFit ha_fit = agr_profile_fit(&ha);
    CHECK(ha_fit.valid);
    CHECK_NEAR(ha_fit.slope, -0.2065f, 1e-3f);

    // ---- far point cannot validate a fit alone ----
    // 2 adjacent cells (i=20,21) → near weight 8 ≥ 6 BUT near span 0.05 < 0.30 → invalid
    AgrProfile fv;
    agr_profile_init(&fv);
    agr_profile_insert(&fv, agr_cell_x(20), 0.0f, 4.0f);
    agr_profile_insert(&fv, agr_cell_x(21), 0.0f, 4.0f);
    for (int i = 0; i < 5; i++) {
        agr_profile_far_hit(&fv, 5.7f, -1.13f);
    }
    CHECK(!agr_profile_fit(&fv).valid);
    // 1 cell + far → near weight 4 < 6 → invalid by weight floor
    AgrProfile fv1;
    agr_profile_init(&fv1);
    agr_profile_insert(&fv1, agr_cell_x(20), 0.0f, 4.0f);
    for (int i = 0; i < 5; i++) {
        agr_profile_far_hit(&fv1, 5.7f, -1.13f);
    }
    CHECK(!agr_profile_fit(&fv1).valid);

    // ---- far point dies when advected out of band ----
    // flat profile i=6..20 (z=0, w=4) + far at (2.6,-0.02): trusted, just past the 2.5 m horizon
    AgrProfile fa;
    agr_profile_init(&fa);
    for (int i = 6; i <= 20; i++) {
        agr_profile_insert(&fa, agr_cell_x(i), 0.0f, 4.0f);
    }
    for (int i = 0; i < 5; i++) {
        agr_profile_far_hit(&fa, 2.6f, -0.02f);
    }
    CHECK(agr_profile_fit(&fa).valid);
    // advance 0.2 m → 4 fwd shifts → far_x advects 2.6 → 2.4 ≤ 2.5 → far_hits reset to 0.
    // cells also shift to i=2..16 (near span 0.7 ≥ 0.3, weight 60 ≥ 6 → still valid).
    agr_profile_advance(&fa, 0.2f);
    CHECK(fa.far_hits == 0);
    AgrFit fa_fit = agr_profile_fit(&fa);
    CHECK(fa_fit.valid);
    CHECK_NEAR(fa_fit.slope, 0.0f, 0.01f);

    // ---- persistence gates jitter, not drift (documented limitation) ----
    // start from a trusted far point (5.7, -0.20); step far_z by -0.29 (< AGR_FAR_DZ=0.30)
    // seven times. each step is consistent → far_hits keeps climbing, never resets.
    // this pins the gate's real guarantee: it rejects jitter, NOT slow drift.
    AgrProfile dr;
    agr_profile_init(&dr);
    for (int i = 0; i < 5; i++) {
        agr_profile_far_hit(&dr, 5.7f, -0.20f);
    }
    float drift_z = -0.20f;
    for (int i = 0; i < 7; i++) {
        drift_z -= 0.29f;
        agr_profile_far_hit(&dr, 5.7f, drift_z);
    }
    CHECK(dr.far_hits >= 12);            // never reset across the whole drift
    CHECK_NEAR(dr.far_z, -2.23f, 1e-3f);  // -0.20 - 7*0.29

    // ---- after far reset, slope returns to flat ----
    // Use profile c (i=6..20, all z=0): far_hits was reset to 1 by the jump test above;
    // reset it fully and confirm fit returns to the original flat slope.
    agr_profile_far_reset(&c);
    AgrFit c_flat = agr_profile_fit(&c);
    CHECK(c_flat.valid);
    CHECK_NEAR(c_flat.slope, 0.0f, 0.01f);

    T_REPORT();
}
