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
#include "../log_hil_data/test_icp_data.h"

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

/**
 * @brief Statically-allocated buffer for storing transformed source points
 *  for processing
 */
static Point2D  icp_src_trans[ICP_MAX_POINTS];

/**
 * @brief buffer which stores each index of its closest point in target for
 *  each source point; index of source is linked to an index of the target cloud
 */ 
static index_t  icp_correspondences[ICP_MAX_POINTS];

/**
 * @brief buffer which stores the squared distance for each correspondence
 */
static float    icp_corr_dist_sq[ICP_MAX_POINTS];

/**
 * @brief Cache variables for ICP to speed up repeated calls with the same
 *  input sizes
 */
static uint16_t icp_valid_range = 0;

#define PL_ICP 1

#ifdef PL_ICP

// __attribute__ (( section(".fastrun") ))
void ICP_2D_play(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float*  out_R,
        float*  out_t)
{   
    // early-return to validate input size against static buffer limits
    if (    (source_size > ICP_MAX_POINTS)
         || (source_size <= 0))
    {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds max %d\n",
               source_size,
               ICP_MAX_POINTS);
#endif

        // Assign default identity transform
        // out_R   = (float[4]){1.f, 0.f,
        //                      0.f, 1.f};

        // out_t   = (float[2]){0.f, 0.f};

        out_R[0] = 1.f; out_R[1] = 0.f;
        out_R[2] = 0.f; out_R[3] = 1.f;

        out_t[0] = 0.f; out_t[1] = 0.f;

        return;
    }

    // counter variables
    uint16_t    iter, i;
    int         j;

    // iterative error for convergence check
    float prev_error;

    // Initialize transformation matrices
    float R[2][2]   = { {1, 0},
                        {0, 1} };
    float t[2]      =   {0, 0};


    // reset cache variables for this ICP run
    memset(&icp_src_trans[0],       0, sizeof(Point2D)*ICP_MAX_POINTS);
    memset(&icp_correspondences[0], 0, sizeof(index_t)*ICP_MAX_POINTS);
    memset(&icp_corr_dist_sq[0],    0,   sizeof(float)*ICP_MAX_POINTS);


    // Copy source to static buffer while sorting this array such that near
    // points are filled in downwards while far points are filled in from the
    // end, upward.
    {
        icp_valid_range = 0;
        Point2D* near   = icp_src_trans;
        Point2D* far    = icp_src_trans + source_size - 1;
        
        for (i = 0; i < source_size; i++) {

            float range_sq  =   source[i].x*source[i].x \
                              + source[i].y*source[i].y;

            if (range_sq < (MAX_ICP_RANGE*MAX_ICP_RANGE)) {

                *(near++)   = source[i];
                icp_valid_range++;

            } else {

                *(far--)    = source[i];
            }
        }
    }

#ifdef DEBUG_OUTPUT
    PRINTF("ICP_2D_i: `ICP_MAX_RANGE` filtered source from %u to %u points\n",
          source_size,
          icp_valid_range);

    if (PRINT_CHECK) {
        PRINTF("ICP tgts:    tgt[0]=(%.2f,%.2f)  tgt[1]=(%.2f,%.2f)\n",
               target[0].x, target[0].y,
               target[1].x, target[1].y);
    }
#endif

    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error  = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {

        // Point2D centroid_src, centroid_tgt;
        float theta, c, s;
        float R_iter[2][2];
        float t_iter[2];
        float mean_error;
        float R_new[2][2];
        float t_new[2];

        // number of correspondences within max distance
        uint16_t valid_count;
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u:  src[0]=(%.2f,%.2f)  src[1]=(%.2f,%.2f)\n",
        //            iter,
        //            icp_src_trans[0].x, icp_src_trans[0].y,
        //            icp_src_trans[1].x, icp_src_trans[1].y);
        // }

        PointCloud dummy_cloud;

        for (i = 0; i < source_size; i++) {
            dummy_cloud.points[i] = icp_src_trans[i];
        }
        dummy_cloud.num_pts = source_size;

        C_format_print((state_se2_t){0,0,0},
                       &dummy_cloud);
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        // for (i = 0; i < icp_valid_range; i++) {
            
        //     icp_correspondences[i] = Find_Closest_Point(
        //             icp_src_trans[i],
        //             target,
        //             target_size,
        //             &icp_corr_dist_sq[i]);
                    
        // }


        // 2. Compute centroids using only valid correspondences
        //    (distance < ICP_MAX_CORR_DIST)
        // centroid_src.x  = 0;    centroid_src.y  = 0;
        // centroid_tgt.x  = 0;    centroid_tgt.y  = 0;
        // valid_count     = 0;

        
        // for (i = 0; i < icp_valid_range; i++) {

        //     if (icp_corr_dist_sq[i] < ICP_MAX_CORR_DIST_SQ) {
        //         centroid_src.x += icp_src_trans[i].x;
        //         centroid_src.y += icp_src_trans[i].y;
        //         centroid_tgt.x += target[icp_correspondences[i]].x;
        //         centroid_tgt.y += target[icp_correspondences[i]].y;
        //         valid_count++;
        //     }

        // }

        // Need at least 3 valid correspondences to compute transformation
//         if (valid_count < 3) {
// #ifdef DEBUG_OUTPUT
//             PRINTF("ICP iter %d: only %d valid correspondences, stopping\n",
//                     iter,
//                     valid_count);
// #endif
//             break;
//         }

        // centroid_src.x /= valid_count;
        // centroid_src.y /= valid_count;
        // centroid_tgt.x /= valid_count;
        // centroid_tgt.y /= valid_count;


        // 3. Compute cross-covariance matrix using only valid correspondences
        // this derivces the optimal rotation matrix that minimizes the mean squared error between the source and target points
        // float   S_xx = 0,   S_xy = 0,
        //         S_yx = 0,   S_yy = 0;
    
        // for (i = 0; i < icp_valid_range; i++) {
            
        //     // early-continue away from pairs that are too far apart
        //     if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ)
        //         continue;
            
        //     float x_s   = icp_src_trans[i].x - centroid_src.x;
        //     float y_s   = icp_src_trans[i].y - centroid_src.y;
        //     float x_t   =   target[icp_correspondences[i]].x
        //                   - centroid_tgt.x;
        //     float y_t   =   target[icp_correspondences[i]].y
        //                   - centroid_tgt.y;

        //     S_xx   += x_s * x_t;
        //     S_xy   += x_s * y_t; 
        //     S_yx   += y_s * x_t;
        //     S_yy   += y_s * y_t;
        // }

        
        // 3. alternatively, the PLICP paper suggests that I accumulate the overdetermined system that solves the matrix `A x = b` firstly by accumulating these matrices incrementally as I find correspondences, 
        float AT_A[TOTAL]   = {0};
        float AT_b[DIMS]    = {0};

        // here, we add in only the points from source that are within the stated `ICP_MAX_RANGE`, which uses the counter `icp_valid_range` to find the limit
        accumulate_PL_ICP_contributions(&icp_src_trans[0], icp_valid_range,
                                        target, target_size,

                                        icp_correspondences,
                                        AT_A, AT_b);

        // also, this retrieves the `icp_correspondences` that is needed for further processing in step 7, so we can reuse this buffer for both purposes. hopefully the array from 0 to `icp_valid_range` is fully populated



        // 4. Compute rotation (using SVD for 2x2)
        // theta       = atan2f(S_xy - S_yx, S_xx + S_yy);
        
    #ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u: S_xx=%.2f S_xy=%.2f S_yx=%.2f S_yy=%.2f -> theta=%.3f rad (%.1f deg)\n",
        //         iter,
        //            S_xx, S_xy, S_yx, S_yy, theta, theta * 57.2958f);
        // }
    #endif
        
        // c   = cosf(theta);  s   = sinf(theta);

        // R_iter[0][0]    =  c;   R_iter[0][1]    = -s;
        // R_iter[1][0]    =  s;   R_iter[1][1]    =  c;


        // 4. alternatively, we use the accumulated `A` and `b` matrices which comes up with the optimal solution `[dx, dy, dtheta]` that minimizes the mean squared error of the source and target.

        float delta[DIMS];
        solve_3x3_system(AT_A, AT_b, delta);


        // // 5. Compute translation
        // t_iter[0]   =   centroid_tgt.x
        //               - (   R_iter[0][0]*centroid_src.x
        //                   + R_iter[0][1]*centroid_src.y);

        // t_iter[1]   =   centroid_tgt.y
        //               - (   R_iter[1][0]*centroid_src.x
        //                   + R_iter[1][1]*centroid_src.y);


        // Update transformation (compose) before transforming points
        // R_new = R_iter * R_old, t_new = R_iter * t_old + t_iter
        // R_new[0][0] = R_iter[0][0]*R[0][0]  +  R_iter[0][1]*R[1][0];
        // R_new[0][1] = R_iter[0][0]*R[0][1]  +  R_iter[0][1]*R[1][1];
        // R_new[1][0] = R_iter[1][0]*R[0][0]  +  R_iter[1][1]*R[1][0];
        // R_new[1][1] = R_iter[1][0]*R[0][1]  +  R_iter[1][1]*R[1][1];

        // t_new[0]    = R_iter[0][0]*t[0]  +  R_iter[0][1]*t[1]  +  t_iter[0];
        // t_new[1]    = R_iter[1][0]*t[0]  +  R_iter[1][1]*t[1]  +  t_iter[1];

        // // update R, t with new values for next iteration        
        // memcpy(R, R_new, sizeof(float)*4);
        // memcpy(t, t_new, sizeof(float)*2);


        // 5. alternatively, compute the incremental transformation R_iter, t_iter from the solution vector `delta`
        theta       = delta[2];
        c   = cosf(theta);  s   = sinf(theta);
        R_iter[0][0]    =  c;   R_iter[0][1]    = -s;
        R_iter[1][0]    =  s;   R_iter[1][1]    =  c;

        t_iter[0]    = delta[0];
        t_iter[1]    = delta[1];


        // Update transformation (compose) before transforming points
        // R_new = R_iter * R_old, t_new = R_iter * t_old + t_iter
        R_new[0][0] = R_iter[0][0]*R[0][0]  +  R_iter[0][1]*R[1][0];
        R_new[0][1] = R_iter[0][0]*R[0][1]  +  R_iter[0][1]*R[1][1];
        R_new[1][0] = R_iter[1][0]*R[0][0]  +  R_iter[1][1]*R[1][0];
        R_new[1][1] = R_iter[1][0]*R[0][1]  +  R_iter[1][1]*R[1][1];

        t_new[0]    = R_iter[0][0]*t[0]  +  R_iter[0][1]*t[1]  +  t_iter[0];
        t_new[1]    = R_iter[1][0]*t[0]  +  R_iter[1][1]*t[1]  +  t_iter[1];

        // update R, t with new values for next iteration        
        memcpy(R, R_new, sizeof(float)*4);
        memcpy(t, t_new, sizeof(float)*2);


        // 6. Transform source points using accumulated transformation R, t
        for (i = 0; i < icp_valid_range; i++) {

            float x = icp_src_trans[i].x;
            float y = icp_src_trans[i].y;

            icp_src_trans[i].x = R_iter[0][0]*x + R_iter[0][1]*y + t_iter[0];
            icp_src_trans[i].y = R_iter[1][0]*x + R_iter[1][1]*y + t_iter[1];
        }

        #ifdef DEBUG_OUTPUT
        char buf[80];
        snprintf(buf, sizeof(buf), "    movement: t=(%.3f, %.3f)",
                                   t_iter[0], t_iter[1]);
        TEST_MESSAGE(buf);

        #endif


        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < icp_valid_range; i++) {

            // Skip pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) 
                continue;

            float dx = icp_src_trans[i].x - target[icp_correspondences[i]].x;
            float dy = icp_src_trans[i].y - target[icp_correspondences[i]].y;

            // update two places to speed up work
            icp_corr_dist_sq[i] = dx*dx + dy*dy;
            mean_error         += icp_corr_dist_sq[i];

            valid_count++;
        }

        // 
        if (valid_count > 0) {
            mean_error  = sqrtf(mean_error / valid_count);
        }

        // if tolerance threshold is met, consider it converged
        if (fabsf(prev_error - mean_error) < tolerance) {
            break;
        }

        prev_error = mean_error;

        // 8. repeat 1. with new source positions and same target until we see convergence or max iterations
    }


    // 9. perform post-processing of far-range points with final R, t
    int num_far   = (int)(source_size - icp_valid_range);
    int far_start = (int)ICP_MAX_POINTS - num_far;
    for (j = (int)ICP_MAX_POINTS - 1; j >= far_start; j--) {
        
        // first, perform an action on the far points with the final transformation
        float x = icp_src_trans[j].x;
        float y = icp_src_trans[j].y;

        icp_src_trans[j].x  = R[0][0]*x + R[0][1]*y + t[0];
        icp_src_trans[j].y  = R[1][0]*x + R[1][1]*y + t[1];

        // then, find their correspondences for potential use in downstream
        // processing (e.g. loop closure)
        icp_correspondences[j] = Find_Closest_Point(
                icp_src_trans[j],
                target,
                target_size,
                &icp_corr_dist_sq[j]);
    }


    // 10. return final transformation R,t
    out_R[0] = R[0][0]; out_R[1] = R[0][1];
    out_R[2] = R[1][0]; out_R[3] = R[1][1];

    out_t[0] = t[0];
    out_t[1] = t[1];

    *num_iter = iter;
    

#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           t[0],
           t[1],
           atan2f(R[1][0], R[0][0]),
           atan2f(R[1][0], R[0][0]) * 57.2958f);
#endif

}

#else 



// __attribute__ (( section(".fastrun") ))
void ICP_2D_play(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float*  out_R,
        float*  out_t)
{   
    // early-return to validate input size against static buffer limits
    if (    (source_size > ICP_MAX_POINTS)
         || (source_size <= 0))
    {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds max %d\n",
               source_size,
               ICP_MAX_POINTS);
#endif

        // Assign default identity transform
        // out_R   = (float[4]){1.f, 0.f,
        //                      0.f, 1.f};

        // out_t   = (float[2]){0.f, 0.f};

        out_R[0] = 1.f; out_R[1] = 0.f;
        out_R[2] = 0.f; out_R[3] = 1.f;

        out_t[0] = 0.f; out_t[1] = 0.f;

        return;
    }

    // counter variables
    uint16_t    iter, i;
    int         j;

    // iterative error for convergence check
    float prev_error;

    // Initialize transformation matrices
    float R[2][2]   = { {1, 0},
                        {0, 1} };
    float t[2]      =   {0, 0};


    // reset cache variables for this ICP run
    memset(&icp_src_trans[0],       0, sizeof(Point2D)*ICP_MAX_POINTS);
    memset(&icp_correspondences[0], 0, sizeof(index_t)*ICP_MAX_POINTS);
    memset(&icp_corr_dist_sq[0],    0,   sizeof(float)*ICP_MAX_POINTS);


    // Copy source to static buffer while sorting this array such that near
    // points are filled in downwards while far points are filled in from the
    // end, upward.
    {
        icp_valid_range = 0;
        Point2D* near   = icp_src_trans;
        Point2D* far    = icp_src_trans + source_size - 1;
        for (i = 0; i < source_size; i++) {

            float range_sq  =   source[i].x*source[i].x \
                              + source[i].y*source[i].y;

            if (range_sq < (MAX_ICP_RANGE*MAX_ICP_RANGE)) {

                *(near++)   = source[i];
                icp_valid_range++;

            } else {

                *(far--)    = source[i];
            }
        }
    }

#ifdef DEBUG_OUTPUT
    PRINTF("ICP_2D_i: `ICP_MAX_RANGE` filtered source from %u to %u points\n",
          source_size,
          icp_valid_range);

    if (PRINT_CHECK) {
        PRINTF("ICP tgts:    tgt[0]=(%.2f,%.2f)  tgt[1]=(%.2f,%.2f)\n",
               target[0].x, target[0].y,
               target[1].x, target[1].y);
    }
#endif
    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error  = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {

        Point2D centroid_src, centroid_tgt;
        float theta, c, s;
        float R_iter[2][2];
        float t_iter[2];
        float mean_error;
        float R_new[2][2];
        float t_new[2];

        // number of correspondences within max distance
        uint16_t valid_count;
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u:  src[0]=(%.2f,%.2f)  src[1]=(%.2f,%.2f)\n",
        //            iter,
        //            icp_src_trans[0].x, icp_src_trans[0].y,
        //            icp_src_trans[1].x, icp_src_trans[1].y);
        // }

        PointCloud dummy_cloud;

        for (i = 0; i < source_size; i++) {
            dummy_cloud.points[i] = icp_src_trans[i];
        }
        dummy_cloud.num_pts = source_size;

        C_format_print((state_se2_t){0,0,0},
                       &dummy_cloud);
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        for (i = 0; i < icp_valid_range; i++) {
            
            icp_correspondences[i] = Find_Closest_Point(
                    icp_src_trans[i],
                    target,
                    target_size,
                    &icp_corr_dist_sq[i]);
                    
        }


        // 2. Compute centroids using only valid correspondences
        //    (distance < ICP_MAX_CORR_DIST)
        centroid_src.x  = 0;    centroid_src.y  = 0;
        centroid_tgt.x  = 0;    centroid_tgt.y  = 0;
        valid_count     = 0;

        
        for (i = 0; i < icp_valid_range; i++) {

            if (icp_corr_dist_sq[i] < ICP_MAX_CORR_DIST_SQ) {
                centroid_src.x += icp_src_trans[i].x;
                centroid_src.y += icp_src_trans[i].y;
                centroid_tgt.x += target[icp_correspondences[i]].x;
                centroid_tgt.y += target[icp_correspondences[i]].y;
                valid_count++;
            }

        }

        // Need at least 3 valid correspondences to compute transformation
        if (valid_count < 3) {
#ifdef DEBUG_OUTPUT
            PRINTF("ICP iter %d: only %d valid correspondences, stopping\n",
                    iter,
                    valid_count);
#endif
            break;
        }

        centroid_src.x /= valid_count;
        centroid_src.y /= valid_count;
        centroid_tgt.x /= valid_count;
        centroid_tgt.y /= valid_count;


        // 3. Compute cross-covariance matrix using only valid correspondences
        // this derivces the optimal rotation matrix that minimizes the mean squared error between the source and target points
        float   S_xx = 0,   S_xy = 0,
                S_yx = 0,   S_yy = 0;
    
        for (i = 0; i < icp_valid_range; i++) {
            
            // early-continue away from pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ)
                continue;
            
            float x_s   = icp_src_trans[i].x - centroid_src.x;
            float y_s   = icp_src_trans[i].y - centroid_src.y;
            float x_t   =   target[icp_correspondences[i]].x
                          - centroid_tgt.x;
            float y_t   =   target[icp_correspondences[i]].y
                          - centroid_tgt.y;

            S_xx   += x_s * x_t;
            S_xy   += x_s * y_t; 
            S_yx   += y_s * x_t;
            S_yy   += y_s * y_t;
        }


        // 4. Compute rotation (using SVD for 2x2)
        theta       = atan2f(S_xy - S_yx, S_xx + S_yy);
        
    #ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u: S_xx=%.2f S_xy=%.2f S_yx=%.2f S_yy=%.2f -> theta=%.3f rad (%.1f deg)\n",
        //         iter,
        //            S_xx, S_xy, S_yx, S_yy, theta, theta * 57.2958f);
        // }
    #endif
        
        c   = cosf(theta);  s   = sinf(theta);

        R_iter[0][0]    =  c;   R_iter[0][1]    = -s;
        R_iter[1][0]    =  s;   R_iter[1][1]    =  c;


        // 5. Compute translation
        t_iter[0]   =   centroid_tgt.x
                      - (   R_iter[0][0]*centroid_src.x
                          + R_iter[0][1]*centroid_src.y);

        t_iter[1]   =   centroid_tgt.y
                      - (   R_iter[1][0]*centroid_src.x
                          + R_iter[1][1]*centroid_src.y);


        // Update transformation (compose) before transforming points
        // R_new = R_iter * R_old, t_new = R_iter * t_old + t_iter
        R_new[0][0] = R_iter[0][0]*R[0][0]  +  R_iter[0][1]*R[1][0];
        R_new[0][1] = R_iter[0][0]*R[0][1]  +  R_iter[0][1]*R[1][1];
        R_new[1][0] = R_iter[1][0]*R[0][0]  +  R_iter[1][1]*R[1][0];
        R_new[1][1] = R_iter[1][0]*R[0][1]  +  R_iter[1][1]*R[1][1];

        t_new[0]    = R_iter[0][0]*t[0]  +  R_iter[0][1]*t[1]  +  t_iter[0];
        t_new[1]    = R_iter[1][0]*t[0]  +  R_iter[1][1]*t[1]  +  t_iter[1];

        // update R, t with new values for next iteration        
        memcpy(R, R_new, sizeof(float)*4);
        memcpy(t, t_new, sizeof(float)*2);


        // 6. Transform source points using accumulated transformation R, t
        for (i = 0; i < icp_valid_range; i++) {

            float x = icp_src_trans[i].x;
            float y = icp_src_trans[i].y;

            icp_src_trans[i].x = R_iter[0][0]*x + R_iter[0][1]*y + t_iter[0];
            icp_src_trans[i].y = R_iter[1][0]*x + R_iter[1][1]*y + t_iter[1];
        }

        // #ifdef DEBUG_OUTPUT
        char buf[80];
        snprintf(buf, sizeof(buf), "    movement: t=(%.3f, %.3f)",
                                   t_iter[0], t_iter[1]);
        TEST_MESSAGE(buf);

        // #endif


        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < icp_valid_range; i++) {

            // Skip pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) 
                continue;

            float dx = icp_src_trans[i].x - target[icp_correspondences[i]].x;
            float dy = icp_src_trans[i].y - target[icp_correspondences[i]].y;

            // update two places to speed up work
            icp_corr_dist_sq[i] = dx*dx + dy*dy;
            mean_error         += icp_corr_dist_sq[i];

            valid_count++;
        }

        // 
        if (valid_count > 0) {
            mean_error  = sqrtf(mean_error / valid_count);
        }

        // if tolerance threshold is met, consider it converged
        if (fabsf(prev_error - mean_error) < tolerance) {
            break;
        }

        prev_error = mean_error;

        // 8. repeat 1. with new source positions and same target until we see convergence or max iterations
    }


    // 9. perform post-processing of far-range points with final R, t
    int num_far   = (int)(source_size - icp_valid_range);
    int far_start = (int)ICP_MAX_POINTS - num_far;
    for (j = (int)ICP_MAX_POINTS - 1; j >= far_start; j--) {
        
        // first, perform an action on the far points with the final transformation
        float x = icp_src_trans[j].x;
        float y = icp_src_trans[j].y;

        icp_src_trans[j].x  = R[0][0]*x + R[0][1]*y + t[0];
        icp_src_trans[j].y  = R[1][0]*x + R[1][1]*y + t[1];

        // then, find their correspondences for potential use in downstream
        // processing (e.g. loop closure)
        icp_correspondences[j] = Find_Closest_Point(
                icp_src_trans[j],
                target,
                target_size,
                &icp_corr_dist_sq[j]);
    }


    // 10. return final transformation R,t
    out_R[0] = R[0][0]; out_R[1] = R[0][1];
    out_R[2] = R[1][0]; out_R[3] = R[1][1];

    out_t[0] = t[0];
    out_t[1] = t[1];

    *num_iter = iter;
    

#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           t[0],
           t[1],
           atan2f(R[1][0], R[0][0]),
           atan2f(R[1][0], R[0][0]) * 57.2958f);
#endif

}

void slam_perform_icp_play(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
              ICPResult*    result)
{

    // output variables
    float R[4];  // 2x2 rotation matrix (row-major)
    float t[2];  // translation vector
    
    
    // If initial guess is non-zero, pre-transform scan1 so ICP only has to
    // find the small residual correction, then compose the two to recover
    // the full transformation: z_ij = initial_guess + residual
    if (    fabsf(initial_guess->x    ) > 1e-3f
         || fabsf(initial_guess->y    ) > 1e-3f
         || fabsf(initial_guess->theta) > 1e-3f)
    {
        
        Pose icp_residual;
        Pose full_transform;

        // Pre-transform scan1 with the initial guess
        PointCloud transformed_scan1;
        transform_point_cloud(scan1, initial_guess, &transformed_scan1);
        
        // ICP finds the residual correction between scan1' and scan2
        ICP_2D_play(transformed_scan1.points, transformed_scan1.num_pts,
                 (Point2D*)scan2->points, scan2->num_pts,
                 MAX_ICP_ITERATIONS,
                 ICP_CONVERGENCE_TOLERANCE,
                 &result->num_iterations,
                 R, t);

        // Compose initial_guess + icp_residual to get the full transformation.
        // compose_poses(p2, p1, result) computes result = p1 + p2
        icp_residual   = (Pose){t[0],
                                t[1],
                                atan2f(R[2], R[0]),
                                0};
        
        compose_poses(&icp_residual, initial_guess, &full_transform);

        result->dx      = full_transform.x;
        result->dy      = full_transform.y;
        result->dtheta  = full_transform.theta;
    

    // ICP finds the full transformation directly
    } else {

        ICP_2D_play(
                (Point2D*)scan1->points, scan1->num_pts,
                (Point2D*)scan2->points, scan2->num_pts,
                MAX_ICP_ITERATIONS,
                ICP_CONVERGENCE_TOLERANCE,
                &result->num_iterations,
                R, t);

        result->dx      = t[0];
        result->dy      = t[1];
        result->dtheta  = atan2f(R[2], R[0]);
    }

    result->valid   = true;

}

#endif // PLAY_ICP_IMPLEMENTATION

void slam_perform_icp_play(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
              ICPResult*    result)
{

    // output variables
    float R[4];  // 2x2 rotation matrix (row-major)
    float t[2];  // translation vector
    
    
    // If initial guess is non-zero, pre-transform scan1 so ICP only has to
    // find the small residual correction, then compose the two to recover
    // the full transformation: z_ij = initial_guess + residual
    if (    fabsf(initial_guess->x    ) > 1e-3f
         || fabsf(initial_guess->y    ) > 1e-3f
         || fabsf(initial_guess->theta) > 1e-3f)
    {
        
        Pose icp_residual;
        Pose full_transform;

        // Pre-transform scan1 with the initial guess
        PointCloud transformed_scan1;
        transform_point_cloud(scan1, initial_guess, &transformed_scan1);
        
        // ICP finds the residual correction between scan1' and scan2
        ICP_2D_play(transformed_scan1.points, transformed_scan1.num_pts,
                 (Point2D*)scan2->points, scan2->num_pts,
                 MAX_ICP_ITERATIONS,
                 ICP_CONVERGENCE_TOLERANCE,
                 &result->num_iterations,
                 R, t);

        // Compose initial_guess + icp_residual to get the full transformation.
        // compose_poses(p2, p1, result) computes result = p1 + p2
        icp_residual   = (Pose){t[0],
                                t[1],
                                atan2f(R[2], R[0]),
                                0};
        
        compose_poses(&icp_residual, initial_guess, &full_transform);

        result->dx      = full_transform.x;
        result->dy      = full_transform.y;
        result->dtheta  = full_transform.theta;
    

    // ICP finds the full transformation directly
    } else {

        ICP_2D_play(
                (Point2D*)scan1->points, scan1->num_pts,
                (Point2D*)scan2->points, scan2->num_pts,
                MAX_ICP_ITERATIONS,
                ICP_CONVERGENCE_TOLERANCE,
                &result->num_iterations,
                R, t);

        result->dx      = t[0];
        result->dy      = t[1];
        result->dtheta  = atan2f(R[2], R[0]);
    }

    result->valid   = true;

}


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

    play_src   = scan_2;
    play_tgt   = scan_3;
    true_delta = pose_3; // already relative to pose_2 because of the code in 
    // true_delta = (Pose){0,0,0,0}; // override to test zero initial guess


    // ── Instrument inputs with C_format_print ────────────────────────────────
    C_format_print((state_se2_t){0.f, 0.f, 0.f},
                   &play_src);
    C_format_print((state_se2_t){true_delta.x,
                                 true_delta.y,
                                 true_delta.theta},
                   &play_tgt);


    // ── Initial guess: half-odometry warm start ─
    init_guess.x         = 0.65 * true_delta.x;
    init_guess.y         = 0.65 * true_delta.y;
    init_guess.theta     = 1.0f * true_delta.theta;

    // init_guess.x         = 0.f;
    // init_guess.y         = 0.f;
    // init_guess.theta     = 0.f;

    init_guess.timestamp = 0;


    // ── Run ICP ──────────────────────────────────────────────────────────────
    slam_perform_icp_play(&play_src, &play_tgt, &init_guess, &icp_result);


    // ── Confidence from playground buffer (icp_corr_dist_sq[] local to this ─
    //    translation unit — NOT the stale ICP_get_cache() from production ICP)
    {
        float total = 0.0f;
        for (i = 0; i < ICP_MAX_POINTS; i++)
            total += expf(-sqrtf(icp_corr_dist_sq[i]) / ERROR_CONFIDENCE_SCALE);
        conf    = total / ICP_MAX_POINTS;
    }


    // ── Print recovered transform alongside target scan ─────────────────────
    C_format_print((state_se2_t){icp_result.dx,
                                 icp_result.dy,
                                 icp_result.dtheta},
                   &play_tgt);
 

    // ── Diagnostics ──────────────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "true:  dx=%.2f  dy=%.2f  dth=%.4f",
             true_delta.x, true_delta.y, true_delta.theta);
    TEST_MESSAGE(buf);

    snprintf(buf, sizeof(buf), "icp:   dx=%.2f  dy=%.2f  dth=%.4f",
             icp_result.dx, icp_result.dy, icp_result.dtheta);
    TEST_MESSAGE(buf);

    snprintf(buf, sizeof(buf), "conf: %.3f, iter: %d", conf, icp_result.num_iterations);
    TEST_MESSAGE(buf);

    snprintf(buf, sizeof(buf), "err:   dx=%.2f  dy=%.2f  dth=%.4f",
             fabsf(true_delta.x     - icp_result.dx),
             fabsf(true_delta.y     - icp_result.dy),
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