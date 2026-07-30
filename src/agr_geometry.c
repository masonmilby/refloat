#include "agr_geometry.h"

void agr_pitch_ring_init(AgrPitchRing *r) {
    for (int i = 0; i < AGR_PITCH_RING; i++) {
        r->pitch[i] = 0.0f;
    }
    r->head = 0;
    r->count = 0;
}

void agr_pitch_ring_push(AgrPitchRing *r, float pitch_rad) {
    r->head = (uint8_t) ((r->head + 1) % AGR_PITCH_RING);
    r->pitch[r->head] = pitch_rad;
    if (r->count < AGR_PITCH_RING) {
        r->count++;
    }
}

float agr_pitch_ring_at(const AgrPitchRing *r, float lag_ticks) {
    if (r->count == 0) {
        return 0.0f;
    }
    float max_lag = (float) (r->count - 1);
    float lag = lag_ticks < 0.0f ? 0.0f : (lag_ticks > max_lag ? max_lag : lag_ticks);
    int i0 = (int) lag;
    float frac = lag - (float) i0;
    int a = (r->head - i0 + AGR_PITCH_RING) % AGR_PITCH_RING;
    int b = (r->head - i0 - 1 + 2 * AGR_PITCH_RING) % AGR_PITCH_RING;
    if (i0 + 1 > (int) r->count - 1) {
        return r->pitch[a];
    }
    return r->pitch[a] * (1.0f - frac) + r->pitch[b] * frac;
}

AgrGroundPoint agr_locate(const AgrGeometry *g, float pitch, float range_m) {
    AgrGroundPoint p = {.ok = false, .x = 0.0f, .z = 0.0f};
    float delta = g->mount_angle_rad + g->mount_offset_rad - pitch;
    if (delta < AGR_MIN_DEPRESSION_RAD) {
        return p;
    }
    float r = range_m - g->range_bias_m;
    if (r <= 0.0f) {
        return p;
    }
    float a = g->mount_height_m - g->wheel_radius_m;
    float xs = g->mount_fwd_m * cosf(pitch) - a * sinf(pitch);
    float zs = g->wheel_radius_m + g->mount_fwd_m * sinf(pitch) + a * cosf(pitch);
    p.x = xs + r * cosf(delta);
    p.z = zs - r * sinf(delta);
    if (p.x < 0.0f || fabsf(p.z) > 2.0f) {
        return p;
    }
    p.ok = true;
    return p;
}
