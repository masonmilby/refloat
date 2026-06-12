// forks/refloat/src/agr_profile.c
#include "agr_profile.h"

void agr_profile_init(AgrProfile *p) {
    for (int i = 0; i < AGR_CELLS; i++) {
        p->cell[i].z = 0.0f;
        p->cell[i].w = 0.0f;
    }
    p->frac_m = 0.0f;
    p->far_x = 0.0f;
    p->far_z = 0.0f;
    p->far_hits = 0;
}

static int cell_index(float x) {
    int i = (int) ((x + AGR_BEHIND_M) / AGR_CELL_M);
    return (x < -AGR_BEHIND_M || i < 0 || i >= AGR_CELLS) ? -1 : i;
}

void agr_profile_insert(AgrProfile *p, float x, float z, float w) {
    int i = cell_index(x);
    if (i < 0 || w <= 0.0f) {
        return;
    }
    // context outlier check: weighted mean of established neighbors (±3 cells)
    float nz = 0.0f, nw = 0.0f;
    int n_est = 0;
    for (int j = i - 3; j <= i + 3; j++) {
        if (j < 0 || j >= AGR_CELLS || j == i) {
            continue;
        }
        if (p->cell[j].w >= AGR_NEIGHBOR_MIN_W) {
            nz += p->cell[j].z * p->cell[j].w;
            nw += p->cell[j].w;
            n_est++;
        }
    }
    if (n_est >= 3 && fabsf(z - nz / nw) > AGR_OUTLIER_DZ) {
        w *= AGR_OUTLIER_SCALE;
    }
    AgrCell *c = &p->cell[i];
    float tw = c->w + w;
    c->z = (c->z * c->w + z * w) / tw;
    c->w = tw > AGR_CELL_W_CAP ? AGR_CELL_W_CAP : tw;
}

void agr_profile_clear_ray(AgrProfile *p, float x0, float z0, float x1, float z1) {
    if (x1 <= x0) {
        return;
    }
    float inv_dx = 1.0f / (x1 - x0);
    // exclusivity is cell-center based: the hit's own cell is swept when the hit lands past its center (harmless below ~45 deg slopes — poke stays under the margin)
    for (int i = 0; i < AGR_CELLS; i++) {
        float x = agr_cell_x(i);
        if (x <= x0 || x >= x1 || p->cell[i].w <= 0.0f) {
            continue;
        }
        float ray_z = z0 + (z1 - z0) * (x - x0) * inv_dx;
        if (p->cell[i].z > ray_z + AGR_CLEAR_MARGIN) {
            p->cell[i].w *= AGR_CLEAR_SCALE;
        }
    }
}
