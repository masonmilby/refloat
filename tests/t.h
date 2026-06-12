// forks/refloat/tests/t.h — minimal plain-assert harness
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int t_failures = 0;
static int t_checks = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        t_checks++;                                                                                \
        if (!(cond)) {                                                                             \
            t_failures++;                                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                                      \
    do {                                                                                           \
        t_checks++;                                                                                \
        if (fabsf((a) - (b)) > (eps)) {                                                            \
            t_failures++;                                                                          \
            printf("FAIL %s:%d: %s=%f != %s=%f\n", __FILE__, __LINE__, #a, (double) (a), #b,       \
                   (double) (b));                                                                  \
        }                                                                                          \
    } while (0)

#define T_REPORT()                                                                                 \
    do {                                                                                           \
        if (t_failures == 0) {                                                                     \
            printf("ALL PASS (%d checks)\n", t_checks);                                            \
            return 0;                                                                              \
        }                                                                                          \
        printf("%d/%d FAILED\n", t_failures, t_checks);                                            \
        return 1;                                                                                  \
    } while (0)
