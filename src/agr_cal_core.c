// forks/refloat/src/agr_cal_core.c
#include "agr_cal_core.h"

#include "agr_math.h"

#include <math.h>

void agr_cal_sweep_init(AgrCalSweep *s) {
    for (int i = 0; i < AGR_CAL_BINS; i++) {
        s->bin[i].sum_r = 0.0f;
        s->bin[i].sum_r2 = 0.0f;
        s->bin[i].n = 0;
    }
    s->active = false;
}

void agr_cal_sweep_add(AgrCalSweep *s, float pitch_rad, float range_m) {
    float pd = pitch_rad * AGR_RAD2DEG;
    int i = (int) ((pd + 40.0f) / AGR_CAL_BIN_DEG);
    if (i < 0 || i >= AGR_CAL_BINS || range_m <= 0.0f) {
        return;
    }
    if (s->bin[i].n < 65535) {
        s->bin[i].sum_r += range_m;
        s->bin[i].sum_r2 += range_m * range_m;
        s->bin[i].n++;
    }
}

static float bin_pitch_rad(int i) {
    return (-40.0f + ((float) i + 0.5f) * AGR_CAL_BIN_DEG) * AGR_DEG2RAD;
}

static bool bin_usable(const AgrCalSweep *s, int i, float mount_angle_cad) {
    return s->bin[i].n >= AGR_CAL_MIN_N &&
           (mount_angle_cad - bin_pitch_rad(i)) * AGR_RAD2DEG >= AGR_CAL_MIN_DELTA_DEG;
}

float agr_cal_model(float pitch, float mount_angle, float theta_err, float h, float f, float b) {
    float delta = mount_angle + theta_err - pitch;
    float a = h - AGR_WHEEL_RADIUS_M;
    float zs = AGR_WHEEL_RADIUS_M + f * sinf(pitch) + a * cosf(pitch);
    return zs / sinf(delta) + b;
}

// solve 4x4 A x = y in place (double precision for numerical stability); returns false if singular
static bool solve4(double A[4][4], double y[4], float x[4]) {
    int idx[4] = {0, 1, 2, 3};
    for (int c = 0; c < 4; c++) {
        int piv = c;
        for (int r = c + 1; r < 4; r++) {
            if (fabs(A[idx[r]][c]) > fabs(A[idx[piv]][c])) {
                piv = r;
            }
        }
        int tmp = idx[c];
        idx[c] = idx[piv];
        idx[piv] = tmp;
        if (fabs(A[idx[c]][c]) < 1e-12) {
            return false;
        }
        for (int r = c + 1; r < 4; r++) {
            double m = A[idx[r]][c] / A[idx[c]][c];
            for (int k = c; k < 4; k++) {
                A[idx[r]][k] -= m * A[idx[c]][k];
            }
            y[idx[r]] -= m * y[idx[c]];
        }
    }
    for (int c = 3; c >= 0; c--) {
        double acc = y[idx[c]];
        for (int k = c + 1; k < 4; k++) {
            acc -= A[idx[c]][k] * x[k];
        }
        x[c] = (float) (acc / A[idx[c]][c]);
    }
    return true;
}

AgrCalResult agr_cal_fit(const AgrCalSweep *s, float mount_angle_cad) {
    AgrCalResult res = {.status = AGR_CAL_SINGULAR, .rms_m = 0.0f, .span_deg = 0.0f, .bins_used = 0};
    // usable bins: enough samples AND world depression >= 8 deg at the prior
    float p_min = 1e9f, p_max = -1e9f;
    int used = 0;
    for (int i = 0; i < AGR_CAL_BINS; i++) {
        if (!bin_usable(s, i, mount_angle_cad)) {
            continue;
        }
        float p = bin_pitch_rad(i);
        used++;
        if (p < p_min) {
            p_min = p;
        }
        if (p > p_max) {
            p_max = p;
        }
    }
    res.bins_used = used;
    res.span_deg = used > 0 ? (p_max - p_min) * AGR_RAD2DEG : 0.0f;
    if (used < 10 || res.span_deg < AGR_CAL_MIN_SPAN_DEG) {
        res.status = AGR_CAL_COVERAGE;
        return res;
    }
    // params: 0=h, 1=f, 2=theta_err(rad), 3=b — start at priors
    float prm[4] = {0.175f, 0.35f, 0.0f, 0.0f};
    const float eps[4] = {1e-4f, 1e-4f, 1e-5f, 1e-4f};
    for (int iter = 0; iter < AGR_CAL_GN_ITERS; iter++) {
        // double precision accumulation to avoid catastrophic cancellation in JtJ
        double JtJ[4][4] = {{0}};
        double Jtr[4] = {0};
        for (int i = 0; i < AGR_CAL_BINS; i++) {
            if (!bin_usable(s, i, mount_angle_cad)) {
                continue;
            }
            float p = bin_pitch_rad(i);
            float w = (float) s->bin[i].n;
            float r_meas = s->bin[i].sum_r / w;
            float pred = agr_cal_model(p, mount_angle_cad, prm[2], prm[0], prm[1], prm[3]);
            float resid = r_meas - pred;
            float J[4];
            for (int k = 0; k < 4; k++) {  // central differences
                float lo[4], hi[4];
                for (int m = 0; m < 4; m++) {
                    lo[m] = prm[m];
                    hi[m] = prm[m];
                }
                lo[k] -= eps[k];
                hi[k] += eps[k];
                float f_lo = agr_cal_model(p, mount_angle_cad, lo[2], lo[0], lo[1], lo[3]);
                float f_hi = agr_cal_model(p, mount_angle_cad, hi[2], hi[0], hi[1], hi[3]);
                J[k] = (f_hi - f_lo) / (2.0f * eps[k]);
            }
            for (int a = 0; a < 4; a++) {
                for (int c = 0; c < 4; c++) {
                    JtJ[a][c] += (double) w * J[a] * J[c];
                }
                Jtr[a] += (double) w * J[a] * resid;
            }
        }
        // Levenberg-Marquardt damping: λ * diag(JtJ) prevents divergence along the
        // near-null direction that arises from the (θ_err, b) aliasing at limited pitch spans.
        for (int k = 0; k < 4; k++) {
            JtJ[k][k] *= 1.001;
        }
        float step[4];
        if (!solve4(JtJ, Jtr, step)) {
            res.status = AGR_CAL_SINGULAR;
            return res;
        }
        for (int k = 0; k < 4; k++) {
            prm[k] += step[k];
        }
    }
    // weighted RMS over used bins (per-bin means)
    float sse = 0.0f, wsum = 0.0f;
    for (int i = 0; i < AGR_CAL_BINS; i++) {
        if (!bin_usable(s, i, mount_angle_cad)) {
            continue;
        }
        float p = bin_pitch_rad(i);
        float w = (float) s->bin[i].n;
        float r = s->bin[i].sum_r / w -
            agr_cal_model(p, mount_angle_cad, prm[2], prm[0], prm[1], prm[3]);
        sse += w * r * r;
        wsum += w;
    }
    res.rms_m = sqrtf(sse / wsum);
    res.mount_height = prm[0];
    res.mount_fwd = prm[1];
    res.mount_offset_deg = prm[2] * AGR_RAD2DEG;
    res.range_bias = prm[3];
    res.status = res.rms_m <= AGR_CAL_MAX_RESID_M ? AGR_CAL_OK : AGR_CAL_RESIDUAL;
    return res;
}
