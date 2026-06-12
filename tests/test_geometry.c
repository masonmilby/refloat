// forks/refloat/tests/test_geometry.c
#include "agr_geometry.h"
#include "agr_math.h"
#include "t.h"

static AgrGeometry flat_geo(void) {
    AgrGeometry g = {
        .mount_angle = 13.0f * AGR_DEG2RAD,
        .mount_offset = 0.0f,
        .mount_height = 0.175f,
        .mount_fwd = 0.35f,
        .range_bias = 0.0f,
    };
    return g;
}

// slant range a sensor with geometry g at pitch p would read on flat ground
static float flat_range(const AgrGeometry *g, float pitch) {
    float delta = g->mount_angle + g->mount_offset - pitch;
    float a = g->mount_height - AGR_WHEEL_RADIUS_M;
    float zs = AGR_WHEEL_RADIUS_M + g->mount_fwd * sinf(pitch) + a * cosf(pitch);
    return zs / sinf(delta) + g->range_bias;
}

int main(void) {
    // pitch ring: fill with a ramp, check interpolation
    AgrPitchRing r;
    agr_pitch_ring_init(&r);
    for (int i = 0; i < 10; i++) {
        agr_pitch_ring_push(&r, 0.01f * i);  // newest = 0.09
    }
    CHECK_NEAR(agr_pitch_ring_at(&r, 0.0f), 0.09f, 1e-6f);
    CHECK_NEAR(agr_pitch_ring_at(&r, 1.0f), 0.08f, 1e-6f);
    CHECK_NEAR(agr_pitch_ring_at(&r, 2.5f), 0.065f, 1e-6f);  // interpolated
    CHECK_NEAR(agr_pitch_ring_at(&r, 99.0f), 0.0f, 1e-6f);   // clamps to oldest

    // locate: flat-ground ranges land at z=0 across the pitch envelope
    AgrGeometry g = flat_geo();
    for (float p = -6.0f; p <= 6.0f; p += 1.0f) {
        float pr = p * AGR_DEG2RAD;
        AgrGroundPoint pt = agr_locate(&g, pr, flat_range(&g, pr));
        CHECK(pt.ok);
        CHECK_NEAR(pt.z, 0.0f, 0.002f);
        CHECK(pt.x > 0.3f && pt.x < 3.0f);
    }

    // range bias round-trips
    g.range_bias = 0.03f;
    AgrGroundPoint pt = agr_locate(&g, 0.0f, flat_range(&g, 0.0f));
    CHECK(pt.ok);
    CHECK_NEAR(pt.z, 0.0f, 0.002f);

    // sanity gates
    g = flat_geo();
    CHECK(!agr_locate(&g, 12.0f * AGR_DEG2RAD, 1.0f).ok);  // delta ~1 deg: near horizon
    CHECK(!agr_locate(&g, 0.0f, 30.0f).ok);                // z way below: |z| > 2 m

    // anchored to hand-computed values (NOT derived from the module's formulas):
    // pitch=0: sensor at (0.35, 0.175); r = 0.175/sin(13deg) = 0.7779467;
    // x = 0.35 + 0.7779467*cos(13deg) = 1.1080056; z = 0
    g = flat_geo();
    pt = agr_locate(&g, 0.0f, 0.7779467f);
    CHECK(pt.ok);
    CHECK_NEAR(pt.x, 1.1080056f, 1e-3f);
    CHECK_NEAR(pt.z, 0.0f, 1e-3f);

    // r <= 0 after bias subtraction rejects
    g.range_bias = 1.0f;
    CHECK(!agr_locate(&g, 0.0f, 0.5f).ok);

    // x < 0 gate (contrived behind-axle mount)
    g = flat_geo();
    g.mount_fwd = -1.0f;
    CHECK(!agr_locate(&g, 0.0f, 0.05f).ok);

    // empty ring returns 0
    AgrPitchRing empty;
    agr_pitch_ring_init(&empty);
    CHECK_NEAR(agr_pitch_ring_at(&empty, 3.0f), 0.0f, 1e-9f);
    T_REPORT();
}
