/**
 * @file test_cases.h
 * @brief Declarations for all Unity test cases and their setup helpers.
 *
 *  All 9 test functions and their supporting init routines are defined in
 *  `test_cases.cpp`. `test_main.cpp` includes this header and uses it as the
 *  sole entry point for `UNITY_BEGIN` / `RUN_TEST` / `UNITY_END`.
 */

// #pragma once

#ifndef __TEST_CASES_H__
#define __TEST_CASES_H__

#ifdef __cplusplus
extern "C" {
#endif


// ────────────────────────────────────────────────────────────────────────────
//  Init helpers (call before UNITY_BEGIN)
// ────────────────────────────────────────────────────────────────────────────

/** 
 * @brief Run ICP (normal + improved) once and cache results/timings.
 */
void init_icp_data(void);

/**
 * @brief Populate the 15-step sequential pose/scan arrays from test_data.h.
 */
void init_pipeline_data(void);


// ────────────────────────────────────────────────────────────────────────────
//  ICP test cases (6)
// ────────────────────────────────────────────────────────────────────────────

void test_icp_returns_valid_result(void);
void test_icp_delta_close_to_known_pose(void);
void test_icp_confidence_above_floor(void);
void test_improved_icp_delta_close_to_known_pose(void);
void test_improved_icp_confidence_above_floor(void);
void test_icp_improved_time_above_unimproved(void);


// ────────────────────────────────────────────────────────────────────────────
//  GraphSLAM pipeline test cases (3)
// ────────────────────────────────────────────────────────────────────────────

void test_graphslam_pose_count(void);
void test_graphslam_loop_closure_fires(void);
void test_graphslam_per_call_timing(void);


// ────────────────────────────────────────────────────────────────────────────
//  Playground: ICP alignment debugger
// ────────────────────────────────────────────────────────────────────────────

void test_icp_alignment_problem(void);


#ifdef __cplusplus
}
#endif

#endif // __TEST_CASES_H__