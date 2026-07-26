#include "vesc_c_if.h"

systime_t host_now_ticks = 0;

static bool host_output_disabled(void) {
    return true;  // suppress log_msg output during tests
}

static float host_system_time(void) {
    return (float) host_now_ticks / SYSTEM_TICK_RATE_HZ;
}

static int host_printf(const char *fmt, ...) {
    (void) fmt;
    return 0;
}

static float host_get_cfg_float(int param) {
    (void) param;
    return 0.2921f;  // 11.5" tire
}

static vesc_if_host host_if = {
    host_output_disabled, host_system_time, host_printf, host_get_cfg_float
};
vesc_if_host *VESC_IF = &host_if;

systime_t vesc_system_time_ticks(void) {
    return host_now_ticks;
}
