#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define AGR_PITCH_RING 32
#define AGR_MIN_DEPRESSION_RAD (2.0f * 0.017453293f)

typedef struct {
    float pitch[AGR_PITCH_RING];
    uint8_t head;
    uint8_t count;
} AgrPitchRing;

typedef struct {
    float mount_angle_rad;
    float mount_offset_rad;
    float mount_height_m;
    float mount_fwd_m;
    float range_bias_m;
    float wheel_radius_m;
} AgrGeometry;

typedef struct {
    bool ok;
    float x;
    float z;
} AgrGroundPoint;

void agr_pitch_ring_init(AgrPitchRing *r);
void agr_pitch_ring_push(AgrPitchRing *r, float pitch_rad);
float agr_pitch_ring_at(const AgrPitchRing *r, float lag_ticks);
AgrGroundPoint agr_locate(const AgrGeometry *g, float pitch_rad, float range_m);
