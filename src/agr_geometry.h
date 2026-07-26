// forks/refloat/src/agr_geometry.h — pure, host-testable
#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define AGR_PITCH_RING 32
#define AGR_MIN_DEPRESSION_RAD (2.0f * 0.017453293f)

typedef struct {
    float pitch[AGR_PITCH_RING];  // radians, gravity-referenced
    uint8_t head;                 // most recent entry
    uint8_t count;
} AgrPitchRing;

typedef struct {
    float mount_angle;   // rad, beam depression below deck plane (CAD prior)
    float mount_offset;  // rad, calibrated residual (incl. live trim)
    float mount_height;  // m, sensor optical center above ground, level deck
    float mount_fwd;     // m, sensor forward of axle
    float range_bias;    // m, sensor zero offset + mount recess
    float wheel_radius;  // m, from the firmware's si_wheel_diameter
} AgrGeometry;

typedef struct {
    bool ok;  // false: geometrically impossible, discard (NOT a quality judgment)
    float x;  // m ahead of the contact patch
    float z;  // m above the contact plane (gravity frame)
} AgrGroundPoint;

void agr_pitch_ring_init(AgrPitchRing *r);
void agr_pitch_ring_push(AgrPitchRing *r, float pitch_rad);
float agr_pitch_ring_at(const AgrPitchRing *r, float lag_ticks);
AgrGroundPoint agr_locate(const AgrGeometry *g, float pitch_rad, float range_m);
