#pragma once
#include "agr_profile.h"
#include "filters/ema.h"

#include <stdbool.h>
#include <stdint.h>

#define AGR_SIGHT_WINDOW 64
#define AGR_RESIDUAL_RECOVER 0.8f

typedef struct {
    float strength_up, strength_down;
    float angle_limit_up_deg, angle_limit_down_deg;
    float taper_erpm;
    float sight_on, sight_off;
    float residual_max_m;
    float fade_rate_per_s;
    float reverse_fade_m;
    float dir_flip_m;
    float rearm_m;
    float rate_limit_deg_s;
} AgrTuning;

typedef struct {
    uint64_t valid_bits;
    uint8_t bits_n;
    bool link_ok, sight_ok, fit_ok, policy_ok;
    bool trusted;
    float fade;
    float rev_m;
    float fwd_m;
    bool dir_forward;
    float valid_fraction;
} AgrTrust;

typedef struct {
    EMA ema;
    float setpoint;
} AgrCond;

void agr_trust_init(AgrTrust *t);
void agr_trust_sample(AgrTrust *t, bool valid);
void agr_trust_update(
    AgrTrust *t, const AgrFit *fit, bool stale, float dist_delta_m, const AgrTuning *cfg, float dt
);
float agr_law(const AgrFit *fit, bool dir_forward, float abs_erpm, const AgrTuning *cfg);
void agr_cond_init(AgrCond *c);
void agr_cond_reset(AgrCond *c);
void agr_cond_configure(AgrCond *c, float cutoff_hz, float frequency);
void agr_cond_update(AgrCond *c, float raw_deg, float fade, const AgrTuning *cfg, float dt);
void agr_cond_winddown(AgrCond *c);
