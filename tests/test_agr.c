// forks/refloat/tests/test_agr.c
#include "agr.h"
#include "t.h"

#include <string.h>

int main(void) {
    AGR agr;
    agr_init(&agr);

    // a well-formed range frame is accepted and lands in the rx ring
    uint8_t frame[AGR_FRAME_DLC] = {0x01, 0x2C, 200, 1};  // 300 mm, strength 200, counter 1
    CHECK(agr_handle_can_frame(&agr, frame, AGR_FRAME_DLC));

    // a short frame is rejected without reading past len
    CHECK(!agr_handle_can_frame(&agr, frame, AGR_FRAME_DLC - 1));

    // default-config output is zero
    CHECK(agr.setpoint == 0.0f);

    // agr_sim must not survive engage: agr_reset runs on every engage
    agr_init(&agr);
    agr.sim_active = true;
    agr.sim_grade = 0.2f;
    agr_reset(&agr);
    CHECK(!agr.sim_active);

    // a frame stamped newer than the latched tick must not wrap the age.
    // Two instances differing only in rx_tick must produce identical profiles.
    AGR a_now, a_ahead;
    // memset before agr_init: agr_init assigns named fields but leaves struct
    // padding indeterminate, and the memcmp below would compare padding too
    memset(&a_now, 0, sizeof a_now);
    memset(&a_ahead, 0, sizeof a_ahead);
    Time t_now = {0};
    MotorData motor = {0};
    IMU imu = {0};
    RefloatConfig conf = {0};
    conf.agr_lag_ms = 10.0f;
    // production default: without depression margin, agr_locate rejects
    // every pitch below and the profile is never touched by ingest
    conf.agr_mount_angle = 13.0f;
    conf.agr_reverse_fade_m = 1.0f;
    conf.agr_filter = 10.0f;
    conf.agr_rate_limit = 15.0f;
    conf.agr_stale_ms = 50.0f;
    conf.agr_residual_max = 0.03f;

    agr_init(&a_now);
    agr_init(&a_ahead);
    agr_configure(&a_now, &conf, 500.0f);
    agr_configure(&a_ahead, &conf, 500.0f);

    // build a pitch ramp so the lag actually selects different ring entries
    for (int i = 0; i < AGR_PITCH_RING; i++) {
        imu.pitch = (float) i * 0.1f;
        t_now.now = 1000 + (systime_t) i * 20;
        host_now_ticks = t_now.now;
        agr_update(&a_now, &motor, &imu, &t_now, &conf, 0.002f);
        agr_update(&a_ahead, &motor, &imu, &t_now, &conf, 0.002f);
    }

    uint8_t rf[AGR_FRAME_DLC] = {0x01, 0x2C, 200, 2};  // 300 mm

    host_now_ticks = t_now.now;  // exactly "now"
    agr_handle_can_frame(&a_now, rf, AGR_FRAME_DLC);
    host_now_ticks = t_now.now + 5;  // stamped after the tick was latched
    agr_handle_can_frame(&a_ahead, rf, AGR_FRAME_DLC);

    agr_update(&a_now, &motor, &imu, &t_now, &conf, 0.002f);
    agr_update(&a_ahead, &motor, &imu, &t_now, &conf, 0.002f);

    CHECK(memcmp(&a_now.profile, &a_ahead.profile, sizeof(AgrProfile)) == 0);

    // staleness: a frame newer than the latched tick is not stale
    AGR s;
    agr_init(&s);
    agr_configure(&s, &conf, 500.0f);
    Time t_s = {0};
    t_s.now = 1000;
    s.last_rx_tick = 1005;  // newer than now
    host_now_ticks = t_s.now;
    agr_update(&s, &motor, &imu, &t_s, &conf, 0.002f);
    CHECK(s.trust.link_ok);

    T_REPORT();
}
