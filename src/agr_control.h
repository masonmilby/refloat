// forks/refloat/src/agr_control.h — pure, host-testable trust + law + conditioning
#pragma once
#include "agr_profile.h"
#include "filters/ema.h"

#include <stdbool.h>
#include <stdint.h>

#define AGR_SIGHT_WINDOW 64
#define AGR_RESIDUAL_CEIL_M 0.03f
#define AGR_RESIDUAL_RECOVER 0.8f
#define AGR_REVERSE_CLEAR_M 0.25f
#define AGR_DIR_DEBOUNCE_ERPM 100.0f

// plain-float mirror of the agr_* config params the control layer needs;
// the glue layer fills it from RefloatConfig each configure()
typedef struct {
    float strength_up, strength_down;
    float angle_limit_up, angle_limit_down;  // deg; down caps nose-down commands
    float taper_erpm;                        // 0 disables
    float sight_on, sight_off;               // valid-fraction hysteresis
    float fade_rate;                         // 1/s
    float reverse_fade_m;
    float rate_limit;  // deg/s
} AgrTuning;

typedef struct {
    // sight window: 1 bit per received range slot (1 = valid return)
    uint64_t valid_bits;
    uint8_t bits_n;
    // gates (post-hysteresis state)
    bool link_ok, sight_ok, fit_ok, policy_ok;
    bool trusted;
    float fade;  // 0..1, slewed
    // reverse policy accumulators
    float reverse_m, forward_m;
    bool dir_forward;  // debounced travel direction
    float valid_fraction;
} AgrTrust;

typedef struct {
    EMA ema;
    float setpoint;  // conditioned output, deg
} AgrCond;

void agr_trust_init(AgrTrust *t);
void agr_trust_sample(AgrTrust *t, bool valid);  // call once per received range slot
// dist_delta_m: signed odometry this tick; stale: link timeout exceeded
void agr_trust_update(
    AgrTrust *t, const AgrFit *fit, bool stale, float erpm, float dist_delta_m,
    const AgrTuning *cfg, float dt
);
// raw law output (deg) from the fit; fit must be valid (caller gates)
float agr_law(const AgrFit *fit, bool dir_forward, float abs_erpm, const AgrTuning *cfg);
void agr_cond_init(AgrCond *c);
void agr_cond_configure(AgrCond *c, float cutoff_hz, float frequency);
void agr_cond_update(AgrCond *c, float raw_deg, float fade, const AgrTuning *cfg, float dt);
void agr_cond_winddown(AgrCond *c);
