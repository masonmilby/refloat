// forks/refloat/tests/test_agr.c
#include "agr.h"
#include "t.h"

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

    T_REPORT();
}
