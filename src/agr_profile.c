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

float agr_profile_grade_at(const AgrProfile *p, float x, bool *ok) {
    float sw = 0.0f, sx = 0.0f, sz = 0.0f, sxx = 0.0f, sxz = 0.0f;
    int n = 0;
    for (int i = 0; i < AGR_CELLS; i++) {
        float cx = agr_cell_x(i);
        if (fabsf(cx - x) > AGR_LOCAL_WIN_M || p->cell[i].w <= 0.0f) {
            continue;
        }
        float w = p->cell[i].w, z = p->cell[i].z;
        sw += w;
        sx += w * cx;
        sz += w * z;
        sxx += w * cx * cx;
        sxz += w * cx * z;
        n++;
    }
    float det = sxx - sx * sx / (sw > 0.0f ? sw : 1.0f);
    if (n < 2 || sw < 2.0f || det < 1e-6f) {
        *ok = false;
        return 0.0f;
    }
    *ok = true;
    return (sxz - sx * sz / sw) / det;
}

// shift the buffer one cell toward the board (forward travel)
static void shift_fwd(AgrProfile *p, float dz) {
    for (int i = 0; i < AGR_CELLS - 1; i++) {
        p->cell[i] = p->cell[i + 1];
        p->cell[i].z -= dz;
    }
    p->cell[AGR_CELLS - 1].z = 0.0f;
    p->cell[AGR_CELLS - 1].w = 0.0f;
}

static void shift_back(AgrProfile *p, float dz) {
    for (int i = AGR_CELLS - 1; i > 0; i--) {
        p->cell[i] = p->cell[i - 1];
        p->cell[i].z += dz;
    }
    p->cell[0].z = 0.0f;
    p->cell[0].w = 0.0f;
}

void agr_profile_far_hit(AgrProfile *p, float x, float z) {
    if (x > AGR_FAR_MAX_M || x <= AGR_AHEAD_M) {
        p->far_hits = 0;
        return;
    }
    if (p->far_hits > 0 && fabsf(z - p->far_z) > AGR_FAR_DZ) {
        p->far_hits = 0;  // inconsistent: restart persistence
    }
    p->far_x = x;
    p->far_z = z;
    if (p->far_hits < 255) {
        p->far_hits++;
    }
}

void agr_profile_far_reset(AgrProfile *p) {
    p->far_hits = 0;
}

AgrFit agr_profile_fit(const AgrProfile *p) {
    AgrFit f = {.valid = false,
                .slope = 0.0f,
                .residual = 0.0f,
                .weight = 0.0f,
                .near_weight = 0.0f,
                .span = 0.0f};
    float sw = 0.0f, sx = 0.0f, sz = 0.0f, sxx = 0.0f, sxz = 0.0f;
    float x_min = 1e9f, x_max = -1e9f;
    for (int i = 0; i < AGR_CELLS; i++) {
        float w = p->cell[i].w;
        if (w <= 0.0f) {
            continue;
        }
        float x = agr_cell_x(i), z = p->cell[i].z;
        sw += w;
        sx += w * x;
        sz += w * z;
        sxx += w * x * x;
        sxz += w * x * z;
        if (x < x_min) {
            x_min = x;
        }
        if (x > x_max) {
            x_max = x;
        }
    }
    // cells must clear both floors on their own — the far point sweetens an
    // already-valid near fit, it never validates one (see Fix 2 in CLAUDE.md)
    if (sw < AGR_FIT_MIN_W) {
        return f;
    }
    f.near_weight = sw;
    f.span = x_max - x_min;
    if (f.span < AGR_FIT_MIN_SPAN) {
        return f;
    }
    bool far = p->far_hits >= AGR_FAR_PERSIST && p->far_x > AGR_AHEAD_M &&
               p->far_x <= AGR_FAR_MAX_M;
    if (far) {
        float w = AGR_FAR_W, x = p->far_x, z = p->far_z;
        sw += w;
        sx += w * x;
        sz += w * z;
        sxx += w * x * x;
        sxz += w * x * z;
    }
    float det = sxx - sx * sx / sw;
    if (det < 1e-6f) {
        return f;
    }
    f.slope = (sxz - sx * sz / sw) / det;
    float intercept = (sz - f.slope * sx) / sw;
    // weighted RMS residual
    float sse = 0.0f;
    for (int i = 0; i < AGR_CELLS; i++) {
        if (p->cell[i].w <= 0.0f) {
            continue;
        }
        float r = p->cell[i].z - (intercept + f.slope * agr_cell_x(i));
        sse += p->cell[i].w * r * r;
    }
    if (far) {
        float r = p->far_z - (intercept + f.slope * p->far_x);
        sse += AGR_FAR_W * r * r;
    }
    f.residual = sqrtf(sse / sw);
    f.weight = sw;
    f.valid = true;
    return f;
}

void agr_profile_advance(AgrProfile *p, float dist_m) {
    p->frac_m += dist_m;
    while (p->frac_m >= AGR_CELL_M || p->frac_m <= -AGR_CELL_M) {
        bool fwd = p->frac_m > 0.0f;
        bool ok = false;
        float slope = agr_profile_grade_at(p, 0.0f, &ok);
        float dz = (ok ? slope : 0.0f) * AGR_CELL_M;
        if (fwd) {
            shift_fwd(p, dz);
            p->frac_m -= AGR_CELL_M;
            if (p->far_hits > 0) {
                p->far_x -= AGR_CELL_M;  // the far hit advects too
                p->far_z -= dz;
            }
        } else {
            shift_back(p, dz);
            p->frac_m += AGR_CELL_M;
            if (p->far_hits > 0) {
                p->far_x += AGR_CELL_M;
                p->far_z += dz;
            }
        }
        // a far point advected out of the trusted band carries a stale z — drop it
        if (p->far_hits > 0 && (p->far_x <= AGR_AHEAD_M || p->far_x > AGR_FAR_MAX_M)) {
            p->far_hits = 0;
        }
    }
}
