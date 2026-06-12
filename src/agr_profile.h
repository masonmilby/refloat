// forks/refloat/src/agr_profile.h — pure, host-testable terrain profile
#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define AGR_CELL_M 0.05f
#define AGR_BEHIND_M 0.25f
#define AGR_AHEAD_M 2.50f
#define AGR_CELLS 55  // (0.25 + 2.50) / 0.05
#define AGR_CELL_W_CAP 8.0f
#define AGR_OUTLIER_DZ 0.10f
#define AGR_OUTLIER_SCALE 0.1f
#define AGR_CLEAR_MARGIN 0.05f
#define AGR_CLEAR_SCALE 0.2f
#define AGR_NEIGHBOR_MIN_W 1.0f
#define AGR_FAR_MAX_M 6.0f
#define AGR_FAR_DZ 0.3f
#define AGR_FAR_PERSIST 5
#define AGR_FAR_W 4.0f
#define AGR_FIT_MIN_W 6.0f
#define AGR_FIT_MIN_SPAN 0.30f
#define AGR_LOCAL_WIN_M 0.30f
#define AGR_CHORD_STALE_SHIFTS 6  // 0.30 m without a fresh near sample → fall back to grade_at

typedef struct {
    float z;  // height above the (current) contact plane, m
    float w;  // confidence weight; 0 = unknown
} AgrCell;

typedef struct {
    AgrCell cell[AGR_CELLS];  // cell i spans x = -AGR_BEHIND_M + i*AGR_CELL_M
    float frac_m;             // sub-cell odometry accumulator
    float far_x, far_z;       // beyond-horizon hit track (virtual fit point)
    uint8_t far_hits;         // consecutive consistent far hits
    float chord_x, chord_z;   // freshest near sample; chord from contact drives advection dz
    uint8_t chord_age;        // shifts since refresh; 255 = none
} AgrProfile;

typedef struct {
    bool valid;       // enough coherent data to act on
    float slope;      // rise/run (tan of grade angle)
    float residual;   // weighted RMS about the line, m
    float weight;     // total fit weight
    float near_weight;  // cells-only weight (far point excluded)
    float span;       // x extent of contributing data, m
} AgrFit;

static inline float agr_cell_x(int i) {
    return -AGR_BEHIND_M + ((float) i + 0.5f) * AGR_CELL_M;
}

void agr_profile_init(AgrProfile *p);
void agr_profile_insert(AgrProfile *p, float x, float z, float w);
// ray from (x0,z0) to the hit at (x1,z1): evict cells contradicted by clearance
void agr_profile_clear_ray(AgrProfile *p, float x0, float z0, float x1, float z1);
void agr_profile_advance(AgrProfile *p, float dist_m);               // Task 4
float agr_profile_grade_at(const AgrProfile *p, float x, bool *ok);  // Task 4
void agr_profile_far_hit(AgrProfile *p, float x, float z);           // Task 5
void agr_profile_far_reset(AgrProfile *p);                           // Task 5
AgrFit agr_profile_fit(const AgrProfile *p);                         // Task 5
