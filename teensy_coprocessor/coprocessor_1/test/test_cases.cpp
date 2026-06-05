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
#include <./config/unity_config.h>

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


#include "test_cases.h"


#include "../log_hil_data/test_data.h"

// this is the test data as captured from a live run
// #include "../log_hil_data/test_icp_data.h"

// this is the test transformations from the icp alignment problem test case
// #include "../log_hil_data/test_icp_transformations.h"



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
#define N_PIPELINE_POSES    11   // POSE_0..POSE_10 / SCAN_0..SCAN_10


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
static uint32_t start_free_running_timer(void)
 * @todo fix this function so it gets rid of the unneccessary `uint32_t` output
 *  as the timer is restarted already by enabling the cycle counter
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

            // init_guess.x         = 0.f;
            // init_guess.y         = 0.f;
            // init_guess.theta     = 0.f;
            // init_guess.timestamp = 0;

            init_guess.x         = 0.5f * delta.x;
            init_guess.y         = 0.5f * delta.y;
            init_guess.theta     = 1.0f * delta.theta;
            init_guess.timestamp = 0;

            
            // init_guess.x         = 1.f * delta.x;
            // init_guess.y         = 1.f * delta.y;
            // init_guess.theta     = 1.f * delta.theta;
            // init_guess.timestamp = 0;

            slam_perform_icp_i(&prev_scan, scan, &init_guess, &icp_result);
            conf     = slam_compute_icp_confidence_i();

            // send message about ICP confidence for debugging
            snprintf(buf, sizeof(buf),
                     "conf: %.3f, iter: %d",
                     conf,
                     icp_result.num_iterations);
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
    g_poses[0]  = pose_0;   g_scans[0]  = &scan_0;
    g_poses[1]  = pose_1;   g_scans[1]  = &scan_1;
    g_poses[2]  = pose_2;   g_scans[2]  = &scan_2;
    g_poses[3]  = pose_3;   g_scans[3]  = &scan_3;
    g_poses[4]  = pose_4;   g_scans[4]  = &scan_4;
    g_poses[5]  = pose_5;   g_scans[5]  = &scan_5;
    g_poses[6]  = pose_6;   g_scans[6]  = &scan_6;
    g_poses[7]  = pose_7;   g_scans[7]  = &scan_7;
    g_poses[8]  = pose_8;   g_scans[8]  = &scan_8;
    g_poses[9]  = pose_9;   g_scans[9]  = &scan_9;
    g_poses[10] = pose_10;  g_scans[10] = &scan_10;
    // g_poses[11] = pose_11;  g_scans[11] = &scan_11;
    // g_poses[12] = pose_12;  g_scans[12] = &scan_12;
    // g_poses[13] = pose_13;  g_scans[13] = &scan_13;
    // g_poses[14] = pose_14;  g_scans[14] = &scan_14;
    // g_poses[15] = pose_15;  g_scans[15] = &scan_15;
    // g_poses[16] = pose_16;  g_scans[16] = &scan_16;
    // g_poses[17] = pose_17;  g_scans[17] = &scan_17;
    // g_poses[18] = pose_18;  g_scans[18] = &scan_18;
    // g_poses[19] = pose_19;  g_scans[19] = &scan_19;
    // g_poses[20] = pose_20;  g_scans[20] = &scan_20;
    // g_poses[21] = pose_21;  g_scans[21] = &scan_21;
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
 * @brief Feed 8 sequential scans (past `MIN_TEMPORAL_GAP + 1 = 6`). Loop-
 *  closure detection fires at steps 6 and 7; whether it actually adds a
 *  constraint depends on dataset geometry and is reported via `TEST_MESSAGE()`
 *  rather than `TEST_ASSERT()`.
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
 *        Reports min / max / avg via `TEST_MESSAGE`; hard-asserts each > 0 µs.
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


// ────────────────────────────────────────────────────────────────────────────
//
//  ALTERNATIVE TEST CASES - SOLVING ICP ALIGNMENT PROBLEM
//
// ────────────────────────────────────────────────────────────────────────────


// ----------------------------------------------------------------------------
//  SCAN PAIR SELECTOR
//
//  Edit these four macros to choose which scans test_icp_alignment_problem()
//  uses.  Two families are available:
//
//  Family A — raw Point2D arrays from test_data.h (default):
//    PLAY_SRC_PTS / PLAY_SRC_POSE   ← source scan + its world-frame pose
//    PLAY_TGT_PTS / PLAY_TGT_POSE   ← target scan + its world-frame pose
//
//  Family B — pre-built PointCloud structs from test_data.h (pipeline data):
//    Comment out the Family A macros and instead assign inside the test:
//      play_src = scan_0;  play_tgt = scan_1;
//    Then set true_delta manually from the known poses.

#define PLAY_SRC_PTS    g_scan_b_points   // ← swap to any Point2D[] in test_data.h
#define PLAY_SRC_POSE   POSE_B            // ← matching world-frame pose
#define PLAY_TGT_PTS    g_scan_c_points
#define PLAY_TGT_POSE   POSE_C


// ----------------------------------------------------------------------------
//  DATA STRUCTURES



void test_icp_alignment_problem(void)
{
    // ── Locals ───────────────────────────────────────────────────────────────
    char        buf[120];
    int         i;
    PointCloud  play_src, play_tgt;
    Pose        true_delta;
    Pose        init_guess;
    Pose        pose_src;
    Pose        pose_tgt;
    ICPResult   icp_result;
    float       conf;
    bool        have_icp    = false;
    memset(&icp_result, 0, sizeof(icp_result));

    // ──────────────────────────────────────────────────────────────────────
    // load_test_scans(PLAY_SRC_PTS, &play_src);
    // load_test_scans(PLAY_TGT_PTS, &play_tgt);


    // ── Ground-truth delta, expressed in target frame ───────
    //  Same formula as test_icp_delta_close_to_known_pose().
    //  For Family B: set true_delta manually from the known pose difference.
    // {
    //     float dx_g  = pose_src.x - pose_tgt.x;
    //     float dy_g  = pose_src.y - pose_tgt.y;
    //     float c     = cosf(-pose_tgt.theta);
    //     float s     = sinf(-pose_tgt.theta);
    //     true_delta.x         = c*dx_g - s*dy_g;
    //     true_delta.y         = s*dx_g + c*dy_g;
    //     true_delta.theta     = normalize_angle(   pose_src.theta
    //                                             - pose_tgt.theta);
    //     true_delta.timestamp = 0;
    // }
    // relative_pose(&pose_src, &pose_tgt, &true_delta);

    play_src   = scan_5;
    play_tgt   = scan_6;
    true_delta = pose_6; // already relative to pose_2 because of the code in test_data.h


    // ── Instrument inputs with C_format_print ───────────────────────────────
#ifdef PROCESSING4_OUTPUT
    C_format_print((state_se2_t){0.f, 0.f, 0.f},
                   &play_src);
    C_format_print((state_se2_t){true_delta.x,
                                 true_delta.y,
                                 true_delta.theta},
                   &play_tgt);
#endif


    // ── Initial guess: odometry warm start ─────────────────────────

    // because we are expressing the delta in terms of target frame, the odometry guess is actually the inverse of the true delta estimate
    // we are also playing it safe by scaling the guess to 50% of the true delta so the ICP doesn't overshoot
    // @todo: will need to transplant this logic into the icp functions themselves so the underlying logic is abstracted away from the test cases and can be used in production code
    init_guess.x         = -0.5f * true_delta.x;
    init_guess.y         = -0.5f * true_delta.y;
    init_guess.theta     = -1.f * true_delta.theta;

    // init_guess.x         = 0.f;
    // init_guess.y         = 0.f;
    // init_guess.theta     = 0.f;

    init_guess.timestamp = 0;


    // ── Run ICP ─────────────────────────────────────────────────────────────
    slam_perform_icp_play(&play_src, &play_tgt, &init_guess, &icp_result);


    // ── Confidence from playground buffer (icp_corr_dist_sq[] local to this ─
    //    translation unit — NOT the stale ICP_get_cache() from production ICP)
    float mean_error = 0.f;
    {
        float total = 0.0f;
        float* icp_corr_dist_sq = ICP_get_cache();

        for (i = 0; i < ICP_MAX_POINTS; i++) {
            mean_error += sqrtf(icp_corr_dist_sq[i]);
            total += expf(-sqrtf(icp_corr_dist_sq[i]) / ERROR_CONFIDENCE_SCALE);
        }
        mean_error /= ICP_MAX_POINTS;
        conf        = total / ICP_MAX_POINTS;
    }


    // ── Print recovered transform alongside target scan ─────────────────────
#ifdef PROCESSING4_OUTPUT
    C_format_print((state_se2_t){icp_result.dx,
                                 icp_result.dy,
                                 icp_result.dtheta},
                   &play_tgt);
#endif


    // ── Diagnostics ──────────────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "true:  dx=%.2f  dy=%.2f  dth=%.4f",
             true_delta.x, true_delta.y, true_delta.theta);
    TEST_MESSAGE(buf);


    snprintf(buf, sizeof(buf), "icp:   dx=%.2f  dy=%.2f  dth=%.4f",
             icp_result.dx, icp_result.dy, icp_result.dtheta);
    TEST_MESSAGE(buf);


    snprintf(buf, sizeof(buf), "mean_error: %.3f, conf: %.3f, iter: %d",
             mean_error, conf, icp_result.num_iterations);
    TEST_MESSAGE(buf);


    snprintf(buf, sizeof(buf), "err:   dx=%.2f  dy=%.2f  dth=%.4f",
             fabsf(true_delta.x     - icp_result.dx),             fabsf(true_delta.y     - icp_result.dy),
             fabsf(true_delta.theta - icp_result.dtheta));
    TEST_MESSAGE(buf);


    // ── Assertions ───────────────────────────────────────────────────────────
    TEST_ASSERT_TRUE(icp_result.valid);

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM,
                             true_delta.x,
                             icp_result.dx);

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM,
                             true_delta.y,
                             icp_result.dy);

    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD,
                             true_delta.theta,
                             icp_result.dtheta);

    // TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(CONFIDENCE_FLOOR, conf);

    have_icp = true;
}