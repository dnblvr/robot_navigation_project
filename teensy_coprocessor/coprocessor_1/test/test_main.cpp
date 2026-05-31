/**
 * @file test_main.cpp
 * @brief Entry point for the GraphSLAM Unity test suite.
 *
 *        All test logic lives in test_cases.cpp / test_cases.h.
 *        This file only handles platform entry points and test sequencing.
 *
 *        9 tests total (subject to change as development progresses)
 *
 *        Environments:
 *          [env:native_sim]    host build, POSIX timing
 *          [env:teensy40_HIL]  hardware-in-the-loop, DWT timing at 600 MHz
 */ 

#include <unity.h>
#include <./config/unity_config.h>

#include "test_cases.h"


void setUp(void)    {}
void tearDown(void) {}

/**
 * @brief shared cross-platform test rig
 */
void test_rig() {

    init_icp_data();
    init_pipeline_data();

    UNITY_BEGIN();

    // ── ICP tests ───────────────────────────────────────────────────────────
    // RUN_TEST(test_icp_returns_valid_result);
    // RUN_TEST(test_icp_delta_close_to_known_pose);
    // RUN_TEST(test_icp_confidence_above_floor);
    // RUN_TEST(test_improved_icp_delta_close_to_known_pose);
    // RUN_TEST(test_improved_icp_confidence_above_floor);
    // RUN_TEST(test_icp_improved_time_above_unimproved);

    // ── GraphSLAM pipeline tests ────────────────────────────────────────────
    // RUN_TEST(test_graphslam_pose_count);
    // RUN_TEST(test_graphslam_loop_closure_fires);

    // result: Expected 0 to be greater than 0       [FAILED]
    // RUN_TEST(test_graphslam_per_call_timing);

    // ── Playground: ICP alignment debugger ─────────────────────────────────
    RUN_TEST(test_icp_alignment_problem);


}


#ifdef __IMXRT1062__   // Teensy 4.0, toolchain-level define, no header needed


extern "C" void setup(void)
{
    test_rig();
    UNITY_END();
}

extern "C" void loop(void) {}

#else   // native / desktop

int main(void)
{
    test_rig();
    return UNITY_END();
}

#endif  // __IMXRT1062__
