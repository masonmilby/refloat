// forks/refloat/src/agr_control.c
#include "agr_control.h"

#include "agr_math.h"

#include <math.h>

void agr_trust_init(AgrTrust *t) {
    t->valid_bits = 0;
    t->bits_n = 0;
    t->link_ok = false;
    t->sight_ok = false;
    t->fit_ok = false;
    t->policy_ok = true;
    t->trusted = false;
    t->fade = 0.0f;
    t->rev_m = 0.0f;
    t->fwd_m = 0.0f;
    t->dir_forward = true;
    t->valid_fraction = 0.0f;
}

void agr_trust_sample(AgrTrust *t, bool valid) {
    t->valid_bits = (t->valid_bits << 1) | (valid ? 1u : 0u);
    if (t->bits_n < AGR_SIGHT_WINDOW) {
        t->bits_n++;
    }
}

static float valid_fraction(const AgrTrust *t) {
    if (t->bits_n == 0) {
        return 0.0f;
    }
    return (float) __builtin_popcountll(t->valid_bits) / (float) t->bits_n;
}

void agr_trust_update(
    AgrTrust *t, const AgrFit *fit, bool stale, float dist_delta_m, const AgrTuning *cfg, float dt
) {
    t->link_ok = !stale;

    t->valid_fraction = valid_fraction(t);
    if (t->valid_fraction <= cfg->sight_off) {
        t->sight_ok = false;
    } else if (t->valid_fraction >= cfg->sight_on && t->bits_n >= AGR_SIGHT_WINDOW / 2) {
        t->sight_ok = true;
    }

    // fit validity is near-field-floored upstream (agr_profile_fit gates on
    // cells-only weight and span): a far point alone can never carry trust
    if (!fit->valid || fit->residual > cfg->residual_max) {
        t->fit_ok = false;
    } else if (fit->residual <= cfg->residual_max * AGR_RESIDUAL_RECOVER) {
        t->fit_ok = true;
    }

    // rev_m is a drawdown: distance below the running forward-most position,
    // clamped to [0, cap] — not a net-since-init sum, so symmetric dither
    // holds it at the current excursion rather than returning it to zero.
    // dist_delta_m is finite- and magnitude-guarded by the caller.
    //
    // dir_forward has its own evidence (fwd_m, contiguous forward progress)
    // rather than sharing rev_m: keying the latch's re-arm on rev_m alone
    // would force it to wait for a full drain — as much forward travel as
    // the reverse took — leaving the wrong (loose) nose-down limit selected
    // the whole time. fwd_m re-arms after a small fixed distance instead, so
    // the tight limit returns promptly regardless of how deep the reversal
    // was; the fade policy (rev_m vs. reverse_fade_m) is unaffected and still
    // needs a full drain to clear.

    // policy evidence: net drawdown (unchanged)
    float cap = cfg->reverse_fade_m + cfg->dir_flip_m;
    t->rev_m = agr_clampf(t->rev_m - dist_delta_m, 0.0f, cap);

    // direction evidence: contiguous forward progress. Capped at the re-arm
    // threshold so it cannot grow without bound over a long ride.
    if (dist_delta_m < 0.0f) {
        t->fwd_m = 0.0f;
    } else {
        t->fwd_m = fminf(t->fwd_m + dist_delta_m, cfg->rearm_m);
    }

    // forward evidence must win outright: checking rev_m first would make
    // the re-arm unreachable while rev_m > flip, forcing the latch to wait
    // for the drawdown to recede to flip on its own -- exactly the
    // full-drain-shaped wait this accumulator split was meant to eliminate
    float flip = fminf(cfg->dir_flip_m, cfg->reverse_fade_m);
    if (t->fwd_m >= cfg->rearm_m) {
        t->dir_forward = true;
    } else if (t->rev_m > flip) {
        t->dir_forward = false;
    }

    // fade policy unchanged
    if (t->rev_m > cfg->reverse_fade_m) {
        t->policy_ok = false;
    } else if (t->rev_m == 0.0f) {
        t->policy_ok = true;
    }

    t->trusted = t->link_ok && t->sight_ok && t->fit_ok && t->policy_ok;
    agr_slewf(&t->fade, t->trusted ? 1.0f : 0.0f, cfg->fade_rate * dt);
}

float agr_law(const AgrFit *fit, bool dir_forward, float abs_erpm, const AgrTuning *cfg) {
    float g_cmd_deg = atanf(fit->slope) * AGR_RAD2DEG;
    bool uphill = dir_forward == (g_cmd_deg > 0.0f);
    float strength = uphill ? cfg->strength_up : cfg->strength_down;
    // the tight limit always caps the travel-leading-end-down direction:
    // board nose-down when forward, board nose-up when reverse
    float lim_pos = dir_forward ? cfg->angle_limit_up : cfg->angle_limit_down;
    float lim_neg = dir_forward ? cfg->angle_limit_down : cfg->angle_limit_up;
    float raw = agr_clampf(strength * g_cmd_deg, -lim_neg, lim_pos);
    if (cfg->taper_erpm > 0.0f) {
        float k = abs_erpm / cfg->taper_erpm;
        raw *= k > 1.0f ? 1.0f : k;
    }
    return raw;
}

void agr_cond_init(AgrCond *c) {
    ema_init(&c->ema);
    ema_reset(&c->ema, 0.0f);
    c->setpoint = 0.0f;
}

void agr_cond_reset(AgrCond *c) {
    ema_reset(&c->ema, 0.0f);
    c->setpoint = 0.0f;
}

void agr_cond_configure(AgrCond *c, float cutoff_hz, float frequency) {
    ema_configure(&c->ema, cutoff_hz, frequency);
}

void agr_cond_update(AgrCond *c, float raw_deg, float fade, const AgrTuning *cfg, float dt) {
    ema_update(&c->ema, raw_deg * fade);
    // the rate limit is the PRIMARY bound on far-point slow drift (persistence
    // in the profile gates jitter only) — survival equipment, never feel
    agr_slewf(&c->setpoint, c->ema.value, cfg->rate_limit * dt);
}

void agr_cond_winddown(AgrCond *c) {
    c->setpoint *= 0.995f;
    c->ema.value *= 0.995f;
}
