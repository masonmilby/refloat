// forks/refloat/tests/test_profile.c
#include "agr_profile.h"
#include "t.h"

static int cell_of(float x) {
    return (int) ((x + AGR_BEHIND_M) / AGR_CELL_M);
}

int main(void) {
    AgrProfile p;
    agr_profile_init(&p);

    // weighted fusion + cap
    agr_profile_insert(&p, 0.50f, 0.10f, 1.0f);
    agr_profile_insert(&p, 0.50f, 0.20f, 1.0f);
    AgrCell *c = &p.cell[cell_of(0.50f)];
    CHECK_NEAR(c->z, 0.15f, 1e-4f);
    CHECK_NEAR(c->w, 2.0f, 1e-4f);
    for (int i = 0; i < 50; i++) {
        agr_profile_insert(&p, 0.50f, 0.15f, 1.0f);
    }
    CHECK(c->w <= AGR_CELL_W_CAP + 1e-4f);

    // out-of-window insert is a no-op
    agr_profile_insert(&p, 5.0f, 0.0f, 1.0f);
    agr_profile_insert(&p, -1.0f, 0.0f, 1.0f);

    // context outlier: flat neighborhood, then a glint 30 cm high
    agr_profile_init(&p);
    for (int i = 13; i <= 18; i++) {
        agr_profile_insert(&p, agr_cell_x(i), 0.0f, 2.0f);
    }
    agr_profile_insert(&p, 0.55f, 0.30f, 1.0f);
    c = &p.cell[cell_of(0.55f)];
    // glint entered at weight 0.1: pulls the established cell barely
    CHECK(c->z < 0.02f);

    // clearance: a cell holding stale high data gets evicted by a ray under it
    agr_profile_init(&p);
    agr_profile_insert(&p, 1.00f, 0.30f, 4.0f);  // stale "wall" at 30 cm
    float w_before = p.cell[cell_of(1.00f)].w;
    // ray from sensor (0.35, 0.175) to a hit on flat ground at (2.0, 0.0):
    // passes ~0.09 m high at x=1.0 — 0.30 m cell pokes above it
    agr_profile_clear_ray(&p, 0.35f, 0.175f, 2.0f, 0.0f);
    CHECK(p.cell[cell_of(1.00f)].w < w_before * 0.5f);
    // a cell at ray height is untouched
    agr_profile_insert(&p, 1.50f, 0.04f, 2.0f);  // ray z at 1.5 ~ 0.053
    float w_keep = p.cell[cell_of(1.50f)].w;
    agr_profile_clear_ray(&p, 0.35f, 0.175f, 2.0f, 0.0f);
    CHECK_NEAR(p.cell[cell_of(1.50f)].w, w_keep, 1e-5f);

    // ---- additional anchored assertions ----

    // exact outlier arithmetic:
    // neighborhood at cells 13..18 (all w=2.0, z=0.0), target cell 16 = cell_of(0.55).
    // glint at 0.55, z=0.30, w=1.0: 5 established neighbors (cells 13-15, 17-18),
    // all w=2.0 >= AGR_NEIGHBOR_MIN_W=1.0, n_est=5>=3.
    // nz/nw = 0.0, |0.30-0.0|=0.30 > AGR_OUTLIER_DZ=0.10 => w_eff=0.1
    // cell 16 was (z=0.0, w=2.0), fusion: tw=2.1, z=(0*2.0+0.30*0.1)/2.1=0.030/2.1
    // expected z = 0.014286, w = 2.1
    agr_profile_init(&p);
    for (int i = 13; i <= 18; i++) {
        agr_profile_insert(&p, agr_cell_x(i), 0.0f, 2.0f);
    }
    agr_profile_insert(&p, 0.55f, 0.30f, 1.0f);
    c = &p.cell[cell_of(0.55f)];
    CHECK_NEAR(c->z, 0.030f / 2.1f, 1e-5f);
    CHECK_NEAR(c->w, 2.1f, 1e-5f);

    // 2-neighbor case: disagreeing sample enters at FULL weight (n_est < 3 rule)
    // Set up cells at i-2 and i+2 relative to target cell_of(1.00)=25.
    // Insert w=2.0 at cell 23 (x=0.90) and cell 27 (x=1.10), leaving cell 25 empty.
    // Then insert at x=1.00, z=0.50, w=1.0 — only 2 established neighbors, no dampening.
    // Expected: cell 25 gets z=0.50, w=1.0 (full weight).
    agr_profile_init(&p);
    agr_profile_insert(&p, agr_cell_x(23), 0.0f, 2.0f);
    agr_profile_insert(&p, agr_cell_x(27), 0.0f, 2.0f);
    agr_profile_insert(&p, 1.00f, 0.50f, 1.0f);
    c = &p.cell[cell_of(1.00f)];
    CHECK_NEAR(c->z, 0.50f, 1e-5f);
    CHECK_NEAR(c->w, 1.0f, 1e-5f);

    // empty-cell glint: glint into an empty cell with >=3 established neighbors becomes
    // z=0.30 exactly (dampened weight 0.1, no prior data to fuse against).
    // cells 30..35 at z=0,w=2.0; cell 33 left empty; insert at cell 33 center.
    agr_profile_init(&p);
    for (int i = 30; i <= 35; i++) {
        if (i == 33) continue;
        agr_profile_insert(&p, agr_cell_x(i), 0.0f, 2.0f);
    }
    agr_profile_insert(&p, agr_cell_x(33), 0.30f, 1.0f);
    c = &p.cell[33];
    CHECK_NEAR(c->z, 0.30f, 1e-6f);
    CHECK_NEAR(c->w, 0.1f, 1e-6f);

    // clear_ray with x1 <= x0 is a no-op (degenerate ray)
    agr_profile_init(&p);
    agr_profile_insert(&p, 1.00f, 0.30f, 3.0f);
    float w_noop = p.cell[cell_of(1.00f)].w;
    agr_profile_clear_ray(&p, 2.0f, 0.0f, 0.5f, 0.1f);  // x1 < x0, degenerate
    CHECK_NEAR(p.cell[cell_of(1.00f)].w, w_noop, 1e-5f);
    agr_profile_clear_ray(&p, 1.0f, 0.1f, 1.0f, 0.1f);  // x1 == x0, degenerate
    CHECK_NEAR(p.cell[cell_of(1.00f)].w, w_noop, 1e-5f);

    // boundary cells: x=-0.25 lands in cell 0; x=2.499 in cell 54; x=2.50 is out
    agr_profile_init(&p);
    agr_profile_insert(&p, -0.25f, 0.05f, 1.0f);
    CHECK_NEAR(p.cell[0].w, 1.0f, 1e-5f);
    agr_profile_insert(&p, 2.499f, 0.07f, 1.0f);
    CHECK_NEAR(p.cell[54].w, 1.0f, 1e-5f);
    agr_profile_insert(&p, 2.50f, 0.10f, 1.0f);  // out of range — should be no-op
    // no cell should have picked up this value; cell 54 still has w=1.0
    CHECK_NEAR(p.cell[54].w, 1.0f, 1e-5f);

    T_REPORT();
}
