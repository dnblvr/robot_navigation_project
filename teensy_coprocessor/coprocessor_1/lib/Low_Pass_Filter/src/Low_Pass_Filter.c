/**
 * @file Low_Pass_Filter.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Low_Pass_Filter.h"

static int filter_taps[LOW_PASS_FILTER_TAP_NUM] = {
    1040,
    2078,
    183,
    -1174,
    846,
    603,
    -1781,
    965,
    1848,
    -3951,
    1037,
    17504,
    17504,
    1037,
    -3951,
    1848,
    965,
    -1781,
    603,
    846,
    -1174,
    183,
    2078,
    1040
};

void Low_Pass_Filter_init(Low_Pass_Filter* f) {
    int i;
    for (i = 0; i < LOW_PASS_FILTER_TAP_NUM; ++i)
        f->history[i] = 0;
    f->last_index = 0;
}

void Low_Pass_Filter_put(Low_Pass_Filter* f, int input) {

    // insert new sample at the current index and increment index
    f->history[f->last_index++] = input;

    // wrap around
    if (f->last_index == LOW_PASS_FILTER_TAP_NUM)
        f->last_index = 0;
}

int Low_Pass_Filter_get(Low_Pass_Filter* f) {

    long long acc   = 0;

    int i;

    int index       = f->last_index;

    for (i = 0; i < LOW_PASS_FILTER_TAP_NUM; ++i) {
        index   = (index != 0) ? (index - 1) : LOW_PASS_FILTER_TAP_NUM - 1;
        acc    += (long long)f->history[index] * filter_taps[i];
    };
    return acc >> 16;
}
