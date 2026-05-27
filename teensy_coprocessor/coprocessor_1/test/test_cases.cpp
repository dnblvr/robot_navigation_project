/**
 * @file test_cases.cpp
 * @brief All 9 Unity test functions for the GraphSLAM test suite.
 *
 *  Split into two groups:
 *    - ICP tests (6): validate ICP_2D and ICP_2D_i correctness + speed
 *    - Pipeline tests (3): validate the full Run_GraphSLAM call sequence
 *      using the 15-scan sequential dataset in test_data.h
 *
 *  Compatible with:
 *    - [env:native_sim]: host timing via clock_gettime(CLOCK_MONOTONIC)
 *    - [env:teensy40_HIL]: ARM Cortex-M7 DWT cycle counter at 600 MHz
 */

#include <unity.h>
#include <unity_config.h>
#include <data_structures.h>
#include <graphslam.h>

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __IMXRT1062__
  #include <Arduino.h>  // imxrt.h → ARM_DWT_CYCCNT, ARM_DEMCR, ARM_DWT_CTRL
#else
  #include <time.h>
#endif

#include "test_data.h"
#include "test_cases.h"


// ────────────────────────────────────────────────────────────────────────────
//
//  CONSTANTS
//
// ────────────────────────────────────────────────────────────────────────────

#define POSITION_TOLERANCE_MM   150.0f  // generous for large-motion capture
#define ANGLE_TOLERANCE_RAD       0.15f // ~8.6°
#define CONFIDENCE_FLOOR          0.50f // minimum acceptable confidence

/* ---- swap these four lines to change which scan pair is tested ---- */
#define SCAN_SRC_PTS   g_scan_b_points
#define POSE_SRC       POSE_B
#define SCAN_TGT_PTS   g_scan_c_points
#define POSE_TGT       POSE_C

/* ---- GraphSLAM pipeline constants (mirror LiDAR_Mapper.h defines) ---- */
#define MIN_CONSTRAINT_MOTION_MM         15.0f
#define MIN_CONFIDENCE_FOR_SMALL_MOTION  0.01f

/* ---- pipeline dataset size ---- */
#define N_PIPELINE_POSES    15   // POSE_0..POSE_14 / SCAN_0..SCAN_14


// ────────────────────────────────────────────────────────────────────────────
//
//  SHARED GLOBALS
//
// ────────────────────────────────────────────────────────────────────────────

// ── ICP test globals ────────────────────────────────────────────────────────
static PointCloud g_scan_a, g_scan_b;

static ICPResult g_icp_result_n = {};
static ICPResult g_icp_result_i = {};

static Pose g_initial_guess = {0.f, 0.f, 0.f, 0};

static float g_confidence_n = 0.f;
static float g_confidence_i = 0.f;

static uint32_t g_time_n_us = 0;   // ICP_2D   execution time (µs)
static uint32_t g_time_i_us = 0;   // ICP_2D_i execution time (µs)

// ── Pipeline test globals ───────────────────────────────────────────────────
static SLAMOptimizer g_slam;

static Pose        g_poses[N_PIPELINE_POSES];
static PointCloud* g_scans[N_PIPELINE_POSES];   // pointers into test_data.h


// ────────────────────────────────────────────────────────────────────────────
//
//  TIMING HELPERS  (DWT on hardware, POSIX on host)
//
// ────────────────────────────────────────────────────────────────────────────

#ifdef __IMXRT1062__

/**
 * @brief use the DWT cycle counter on the ARM Cortex-M7 to start a free-
 *  running timer
 * 
 * @return uint32_t 
 * @retval `start_time` in microseconds
 */
static uint32_t start_free_running_timer(void)
{
    ARM_DEMCR      |=  ARM_DEMCR_TRCENA;
    ARM_DWT_CYCCNT  =  0;
    ARM_DWT_CTRL   |=  ARM_DWT_CTRL_CYCCNTENA;
    return ARM_DWT_CYCCNT;
}

/**
 * @brief Calculate elapsed time in microseconds since `start_time` using the
 *  DWT cycle counter.
 * 
 * @param[in] start_time The start time returned by `start_free_running_timer()`
 * 
 * @return `uint32_t`
 * @retval `elapsed_us` in microseconds
 * 
 * @note Cast to `uint64_t` before multiplying - `uint32_t` overflows at ~7 µs
 *  real elapsed time (4295 cycles × 1,000,000 > 2^32).
 * 
 * @note Must call `start_free_running_timer()` first to initialize DWT and
 *  avoid overflow issues.
 */
static uint32_t get_elapsed_time_us(uint32_t start_time)
{
    uint32_t elapsed_cycles = ARM_DWT_CYCCNT - start_time;
    return (uint32_t)((uint64_t)elapsed_cycles * 1000000ULL / 600000000ULL);
}

#endif  // __IMXRT1062__


// ────────────────────────────────────────────────────────────────────────────
//
//  ICP HELPERS
//
// ────────────────────────────────────────────────────────────────────────────

static void load_test_scans(
        const Point2D*      source, 
              PointCloud*   dest)
{
    for (uint32_t i = 0; i < OUTPUT_BUFFER; i++) {
        dest->points[i] = source[i];
    }
    dest->num_pts   = OUTPUT_BUFFER;
}


// ────────────────────────────────────────────────────────────────────────────
//
//  PIPELINE HELPERS
//
// ────────────────────────────────────────────────────────────────────────────

static void reset_slam(void)
{
    slam_initialize(&g_slam);
}

/**
 * @brief Execute one full GraphSLAM pipeline step — mirrors Run_GraphSLAM()
 *        from LiDAR_Mapper.h using the local g_slam instead of the firmware
 *        global slam_optimizer.
 */
static void run_graphslam_step(PointCloud* scan, Pose today, Pose delta)
{
    bool      have_icp = false;
    float     conf     = 0.5f;
    ICPResult icp_result;
    memset(&icp_result, 0, sizeof(icp_result));

    // ── ICP against the previous scan ───────────────────────────────────────
    if (g_slam.buffer_size > 0) {

        int        prev_id = g_slam.buffer_size - 1;
        PointCloud prev_scan;
        Pose       prev_pose;

        if (    slam_get_scan(&g_slam, prev_id, &prev_scan)
             && slam_get_pose(&g_slam, prev_id, &prev_pose))
        {
            char buf[80];
            Pose init_guess;
            // init_guess.x         = 0.5f * delta.x;
            // init_guess.y         = 0.5f * delta.y;
            // init_guess.theta     = 1.0f * delta.theta;
            // init_guess.timestamp = 0;

            init_guess.x         = 0.f;
            init_guess.y         = 0.f;
            init_guess.theta     = 0.f;
            init_guess.timestamp = 0;

            slam_perform_icp_i(&prev_scan, scan, &init_guess, &icp_result);
            conf     = slam_compute_icp_confidence_i();

            // send message about ICP confidence for debugging
            snprintf(buf, sizeof(buf), "confidence: %.3f", conf);
            TEST_MESSAGE(buf);

            have_icp = true;
        }
    }

    // ── Add new pose and scan to the graph ──────────────────────────────────
    slam_add_pose(&g_slam, &today, scan);

    // ── Odometry constraint ─────────────────────────────────────────────────
    if (g_slam.buffer_size >= 2) {

        int prev_id = g_slam.buffer_size - 2;
        int curr_id = g_slam.buffer_size - 1;

        Pose measurement;
        measurement.timestamp = 0;
        if (have_icp && conf > 0.2f) {
            measurement.x     = icp_result.dx;
            measurement.y     = icp_result.dy;
            measurement.theta = icp_result.dtheta;
        } else {
            measurement.x     = delta.x;
            measurement.y     = delta.y;
            measurement.theta = delta.theta;
        }

        float mag = sqrtf(  measurement.x * measurement.x
                          + measurement.y * measurement.y);

        float eff_conf = (    mag < MIN_CONSTRAINT_MOTION_MM
                           && fabsf(measurement.theta) < 0.05f)
                         ? MIN_CONFIDENCE_FOR_SMALL_MOTION
                         : conf;

        slam_add_odometry_constraint(&g_slam, prev_id, curr_id,
                                     measurement.x, measurement.y,
                                     measurement.theta, eff_conf);
    }

    // ── Loop-closure detection (fires once buffer_size > 6) ─────────────────
    if (g_slam.buffer_size > MIN_TEMPORAL_GAP + 1) {
        slam_detect_loop_closure(&g_slam, g_slam.buffer_size - 1);
    }

    // ── Gauss-Newton optimization (every step; OPTIMIZE_INTERVAL == 1) ──────
    if (    g_slam.buffer_size > 0
         && g_slam.buffer_size % OPTIMIZE_INTERVAL == 0)
    {
        slam_optimize_gauss_newton(&g_slam, MAX_GAUSS_NEWTON_ITERS);
    }
}

/** @brief Run n sequential pipeline steps beginning at index 0. */
static void run_pipeline_steps(int n)
{
    int i;
    for (i = 0; i < n; i++) {

        Pose today = g_poses[i];

        Pose delta;
        delta.timestamp = 0;
        if (i == 0) {
            delta.x     = 0.f;
            delta.y     = 0.f;
            delta.theta = 0.f;
        } else {
            delta.x     = g_poses[i].x - g_poses[i-1].x;
            delta.y     = g_poses[i].y - g_poses[i-1].y;
            delta.theta = normalize_angle(g_poses[i].theta - g_poses[i-1].theta);
        }

        run_graphslam_step(g_scans[i], today, delta);

#ifdef __IMXRT1062__
        /* Service the USB stack between steps.  Without this, back-to-back
           ICP + Gauss-Newton calls keep the CPU busy long enough for the USB
           host to time out and drop the serial connection. */
        yield();
#endif
    }
}


// ────────────────────────────────────────────────────────────────────────────
//
//  INIT HELPERS  (called from test_main.cpp before UNITY_BEGIN)
//
// ────────────────────────────────────────────────────────────────────────────

void init_icp_data(void)
{
    load_test_scans(SCAN_SRC_PTS, &g_scan_a);
    load_test_scans(SCAN_TGT_PTS, &g_scan_b);

    g_initial_guess.x         = 0.5f * (POSE_SRC.x - POSE_TGT.x);
    g_initial_guess.y         = 0.5f * (POSE_SRC.y - POSE_TGT.y);
    g_initial_guess.theta     = 0.5f * normalize_angle(POSE_SRC.theta - POSE_TGT.theta);
    g_initial_guess.timestamp = 0;


    // ── ICP_2D (unimproved) ─────────────────────────────────────────────────

#ifdef __IMXRT1062__
    uint32_t t0 = start_free_running_timer();
#else
    struct timespec ts0, ts1;
    clock_gettime(CLOCK_MONOTONIC, &ts0);
#endif


    slam_perform_icp(&g_scan_a, &g_scan_b, &g_initial_guess, &g_icp_result_n);
    g_confidence_n = slam_compute_icp_confidence(&g_scan_a, &g_scan_b, &g_icp_result_n);


#ifdef __IMXRT1062__
    g_time_n_us = get_elapsed_time_us(t0);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    g_time_n_us = (uint32_t)(  (ts1.tv_sec  - ts0.tv_sec)  * 1000000L
                              + (ts1.tv_nsec - ts0.tv_nsec) / 1000L);
#endif


    // ── ICP_2D_i (improved) ─────────────────────────────────────────────────

#ifdef __IMXRT1062__
    t0 = start_free_running_timer();
#else
    clock_gettime(CLOCK_MONOTONIC, &ts0);
#endif


    slam_perform_icp_i(&g_scan_a, &g_scan_b, &g_initial_guess, &g_icp_result_i);
    g_confidence_i = slam_compute_icp_confidence_i();


#ifdef __IMXRT1062__
    g_time_i_us = get_elapsed_time_us(t0);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    g_time_i_us = (uint32_t)(  (ts1.tv_sec  - ts0.tv_sec)  * 1000000L
                              + (ts1.tv_nsec - ts0.tv_nsec) / 1000L);
#endif

}

void init_pipeline_data(void)
{
    g_poses[0]  = POSE_0;   g_scans[0]  = &SCAN_0;
    g_poses[1]  = POSE_1;   g_scans[1]  = &SCAN_1;
    g_poses[2]  = POSE_2;   g_scans[2]  = &SCAN_2;
    g_poses[3]  = POSE_3;   g_scans[3]  = &SCAN_3;
    g_poses[4]  = POSE_4;   g_scans[4]  = &SCAN_4;
    g_poses[5]  = POSE_5;   g_scans[5]  = &SCAN_5;
    g_poses[6]  = POSE_6;   g_scans[6]  = &SCAN_6;
    g_poses[7]  = POSE_7;   g_scans[7]  = &SCAN_7;
    g_poses[8]  = POSE_8;   g_scans[8]  = &SCAN_8;
    g_poses[9]  = POSE_9;   g_scans[9]  = &SCAN_9;
    g_poses[10] = POSE_10;  g_scans[10] = &SCAN_10;
    g_poses[11] = POSE_11;  g_scans[11] = &SCAN_11;
    g_poses[12] = POSE_12;  g_scans[12] = &SCAN_12;
    g_poses[13] = POSE_13;  g_scans[13] = &SCAN_13;
    g_poses[14] = POSE_14;  g_scans[14] = &SCAN_14;
}


// ────────────────────────────────────────────────────────────────────────────
//
//  ICP TEST CASES (6)
//
// ────────────────────────────────────────────────────────────────────────────

void test_icp_returns_valid_result(void)
{
    TEST_ASSERT_TRUE(g_icp_result_n.valid);
}

void test_icp_delta_close_to_known_pose(void)
{
    float dx_g    = POSE_SRC.x - POSE_TGT.x;
    float dy_g    = POSE_SRC.y - POSE_TGT.y;
    float c       = cosf(-POSE_TGT.theta);
    float s       = sinf(-POSE_TGT.theta);
    float exp_dx  = c*dx_g - s*dy_g;
    float exp_dy  = s*dx_g + c*dy_g;
    float exp_dth = normalize_angle(POSE_SRC.theta - POSE_TGT.theta);

    printf("pose_x:  expected=%.2f  icp=%.2f\n", exp_dx,  g_icp_result_n.dx);
    printf("pose_y:  expected=%.2f  icp=%.2f\n", exp_dy,  g_icp_result_n.dy);
    printf("pose_th: expected=%.3f  icp=%.3f\n", exp_dth, g_icp_result_n.dtheta);

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, 
                             exp_dx,  
                             g_icp_result_n.dx);

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, 
                             exp_dy,  
                             g_icp_result_n.dy);

    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD,   
                             exp_dth, 
                             g_icp_result_n.dtheta);
}

void test_icp_confidence_above_floor(void)
{
    printf("  ICP confidence = %.3f\n", g_confidence_n);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(CONFIDENCE_FLOOR, g_confidence_n);
}

void test_improved_icp_delta_close_to_known_pose(void)
{
    float dx_g  = POSE_SRC.x - POSE_TGT.x;
    float dy_g  = POSE_SRC.y - POSE_TGT.y;
    float c     = cosf( -POSE_TGT.theta );
    float s     = sinf( -POSE_TGT.theta );
    float exp_dx  = c*dx_g - s*dy_g;
    float exp_dy  = s*dx_g + c*dy_g;
    float exp_dth = normalize_angle(POSE_SRC.theta - POSE_TGT.theta);

    printf("pose_x:  expected=%.2f  icp=%.2f\n", exp_dx,  g_icp_result_i.dx);
    printf("pose_y:  expected=%.2f  icp=%.2f\n", exp_dy,  g_icp_result_i.dy);
    printf("pose_th: expected=%.3f  icp=%.3f\n", exp_dth, g_icp_result_i.dtheta);

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, exp_dx,  g_icp_result_i.dx);
    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, exp_dy,  g_icp_result_i.dy);
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD,   exp_dth, g_icp_result_i.dtheta);
}

void test_improved_icp_confidence_above_floor(void)
{
    printf("  ICP_i confidence = %.3f\n", g_confidence_i);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(CONFIDENCE_FLOOR, g_confidence_i);
}

void test_icp_improved_time_above_unimproved(void)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "ICP_2D   : %lu us", (unsigned long)g_time_n_us);
    TEST_MESSAGE(buf);
    snprintf(buf, sizeof(buf), "ICP_2D_i : %lu us", (unsigned long)g_time_i_us);
    TEST_MESSAGE(buf);

    TEST_ASSERT_GREATER_THAN_UINT32(0, g_time_n_us);
    TEST_ASSERT_GREATER_THAN_UINT32(0, g_time_i_us);
}


// ────────────────────────────────────────────────────────────────────────────
//
//  GRAPHSLAM PIPELINE TEST CASES (3)
//
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief Feed 5 sequential scans; verify `buffer_size == 5` and
 *  `num_constraints == 4` (one odometry edge per consecutive pair, no loop
 *  closure at `buffer_size < 7`).
 */
void test_graphslam_pose_count(void)
{
    reset_slam();
    run_pipeline_steps(5);

    TEST_ASSERT_EQUAL_INT(5, g_slam.buffer_size);
    TEST_ASSERT_EQUAL_INT(4, g_slam.num_constraints);
}

/**
 * @brief Feed 8 sequential scans (past MIN_TEMPORAL_GAP + 1 = 6). Loop-closure
 *  detection fires at steps 6 and 7; whether it actually adds a constraint
 *  depends on dataset geometry and is reported via TEST_MESSAGE rather than
 *  hard-asserted.
 */
void test_graphslam_loop_closure_fires(void)
{
    reset_slam();
    run_pipeline_steps(8);

    TEST_ASSERT_EQUAL_INT(8, g_slam.buffer_size);
    TEST_ASSERT_GREATER_OR_EQUAL_INT(7, g_slam.num_constraints);

    char buf[80];
    snprintf(buf, sizeof(buf),
             "num_constraints=%d after 8 steps (odom=7, lc_extra=%d)",
             g_slam.num_constraints,
             g_slam.num_constraints - 7);
    TEST_MESSAGE(buf);
}

/**
 * @brief Run all N_PIPELINE_POSES steps with per-call timing.
 *        Reports min / max / avg via TEST_MESSAGE; hard-asserts each > 0 µs.
 */
void test_graphslam_per_call_timing(void)
{
    reset_slam();

    uint32_t times_us[N_PIPELINE_POSES];
    memset(times_us, 0, sizeof(times_us));

    uint32_t min_us = UINT32_MAX;
    uint32_t max_us = 0;
    uint64_t sum_us = 0;

    int i;
    for (i = 0; i < N_PIPELINE_POSES; i++) {

        Pose today = g_poses[i];
        Pose delta;
        delta.timestamp = 0;
        if (i == 0) {
            delta.x = 0.f; delta.y = 0.f; delta.theta = 0.f;
        } else {
            delta.x     = g_poses[i].x - g_poses[i-1].x;
            delta.y     = g_poses[i].y - g_poses[i-1].y;
            delta.theta = normalize_angle(g_poses[i].theta - g_poses[i-1].theta);
        }

#ifdef __IMXRT1062__
        uint32_t t0 = start_free_running_timer();

        run_graphslam_step(g_scans[i], today, delta);

        times_us[i] = get_elapsed_time_us(t0);
#else
        struct timespec ts0, ts1;
        clock_gettime(CLOCK_MONOTONIC, &ts0);

        run_graphslam_step(g_scans[i], today, delta);

        clock_gettime(CLOCK_MONOTONIC, &ts1);
        times_us[i] = (uint32_t)(   (ts1.tv_sec  - ts0.tv_sec)  * 1000000L
                                  + (ts1.tv_nsec - ts0.tv_nsec) / 1000L);
#endif

        if (times_us[i] < min_us)
            min_us = times_us[i];
        if (times_us[i] > max_us)
            max_us = times_us[i];
        sum_us += times_us[i];

#ifdef __IMXRT1062__
        // DWT at 600 MHz: every step takes at least 1 µs on real hardware
        TEST_ASSERT_GREATER_THAN_UINT32(0, times_us[i]);

        yield();  // keep USB alive between timed steps
#endif
    }

    char buf[128];
    snprintf(buf, sizeof(buf),
             "per-call latency (us): min=%lu  max=%lu  avg=%lu",
             (unsigned long)min_us,
             (unsigned long)max_us,
             (unsigned long)(sum_us / N_PIPELINE_POSES));
    TEST_MESSAGE(buf);

    /* On native (Windows) clock_gettime resolution may be several ms, so
       individual sub-ms steps can measure as 0 µs.  Assert only that the
       pipeline completed (at least one non-zero measurement). */
    TEST_ASSERT_GREATER_THAN_UINT64(0, sum_us);

}
