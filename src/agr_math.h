#pragma once

#define AGR_DEG2RAD 0.017453293f
#define AGR_RAD2DEG 57.29578f

static inline float agr_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float agr_signf(float v) {
    return v < 0.0f ? -1.0f : 1.0f;
}

static inline void agr_slewf(float *value, float target, float step) {
    if (target > *value + step) {
        *value += step;
    } else if (target < *value - step) {
        *value -= step;
    } else {
        *value = target;
    }
}
