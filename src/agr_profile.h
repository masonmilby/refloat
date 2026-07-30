#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define AGR_CELL_M 0.05f
#define AGR_BEHIND_M 0.25f
#define AGR_AHEAD_M 2.50f
#define AGR_CELLS 55
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
#define AGR_CHORD_STALE_SHIFTS 6

typedef struct {
    float z;
    float w;
} AgrCell;

typedef struct {
    AgrCell cell[AGR_CELLS];
    float frac_m;
    float far_x, far_z;
    uint8_t far_hits;
    float chord_x, chord_z;
    uint8_t chord_age;
} AgrProfile;

typedef struct {
    bool valid;
    float slope;
    float residual;
    float weight;
    float near_weight;
    float span;
} AgrFit;

static inline float agr_cell_x(int i) {
    return -AGR_BEHIND_M + ((float) i + 0.5f) * AGR_CELL_M;
}

void agr_profile_init(AgrProfile *p);
void agr_profile_insert(AgrProfile *p, float x, float z, float w);
void agr_profile_clear_ray(AgrProfile *p, float x0, float z0, float x1, float z1);
void agr_profile_advance(AgrProfile *p, float dist_m);
float agr_profile_grade_at(const AgrProfile *p, float x, bool *ok);
void agr_profile_far_hit(AgrProfile *p, float x, float z);
void agr_profile_far_reset(AgrProfile *p);
AgrFit agr_profile_fit(const AgrProfile *p);
