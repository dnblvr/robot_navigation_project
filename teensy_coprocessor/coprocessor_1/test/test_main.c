/**
 * @file test_main.c
 * @author your name (you@domain.com)
 * @brief test harness for GraphSLAM unit tests using Unity framework
 * @version 0.1
 * @date 2026-05-16
 */
#include <unity.h>
#include <unity_config.h>
#include <data_structures.h>
#include <graphslam.h>
#include <stdio.h>
#include <time.h>


// ----------------------------------------------------------------------------
//
//  CONSTANTS & TEST DATA
//
// ----------------------------------------------------------------------------

#define POSITION_TOLERANCE_MM   150.0f   // generous for large-motion capture
#define ANGLE_TOLERANCE_RAD     0.15f    // ~8.6°
#define CONFIDENCE_FLOOR        0.20f    // minimum acceptable confidence

/**
 * @brief Known scan data a captured at a specific pose, used for testing ICP alignment.
 */
static Point2D g_scan_a_points[OUTPUT_BUFFER] = {
    {-154.79,	-654.96	},
    {-295.26,	-1072.6	},
    {-216.48,	-630.37	},
    {-253.56,	-560.84	},
    {-260.83,	-533.67	},
    {-284.94,	-498.86	},
    {-303.3,	-469.56	},
    {-320.43,	-461.09	},
    {-1282.39,	-1340.33},
    {-196.32,	-167.4	},
    {-208.97,	-157.2	},
    {-225.44,	-159.23	},
    {-232.23,	-142.57	},
    {-235.97,	-135.29	},
    {-234.12,	-115.37	},
    {-240.86,	-99.23	},
    {-247.52,	-93.24	},
    {-254.88,	-77.85	},
    {-263.85,	-72.01	},
    {-272.37,	-55.65	},
    {-283.67,	-49.78	},
    {-293.82,	-31.45	},
    {-307.17,	-14.17	},
    {-317.96,	-5.29	},
    {-332.1,	16.41	},
    {-345.92,	27.32	},
    {-356.9,	50.85	},
    {-375.27,	79.02	},
    {-392.4,	100.23	},
    {-408.21,	130.3	},
    {-437.81,	155.17	},
    {-462.77,	198.38	},
    {-493.87,	230.46	},
    {-574.61,	338.68	},
    {-1511.06,	1150.99	},
    {-1410.36,	1144	},
    {-1163.16,	1136.19	},
    {-2330.72,	2595.64	},
    {-1185.81,	1835.83	},
    {-1827.16,	3009.22	},
    {-1632.67,	3134.25	},
    {-1544.24,	3190.42	},
    {-283.12,	846.93	},
    {-242.6,	797.41	},
    {-117.41,	573.1	},
    {-71.32,	512.56	},
    {-34.34,	470.75	},
    {-18.91,	484.63	},
    {11.09,	    501.88	},
    {27.61,	    518.77	},
    {63.32,	    536.27	},
    {83.64,	    562.82	},
    {170.66,	612.15	},
    {201.6,	    643.67	},
    {280.74,	668.98	},
    {311.47,	628.56	},
    {321.09,	601.12	},
    {363.73,	584.58	},
    {476.89,	550.72	},
    {484.29,	525.33	},
    {509.11,	485.51	},
    {533.18,	448.14	},
    {555.58,	437.25	},
    {975.01,	627.53	},
    {1060.66,	590.58	},
    {5665.08,	2490.85	},
    {4363.64,	1442.87	},
    {3392.16,	882.21	},
    {3079.07,	500.6	},
    {3271.65,	431.63	},
    {3496.16,	241.6	},
    {3585.44,	20.53	},
    {2982.46,   -268.31	},
    {3027.47,   -357.49	},
    {3031.53,   -561.01	},
    {2894.53,   -630.94	},
    {2987.1,    -853.89	},
    {4493.42,   -1603.63},
    {1427.7,    -558.34	},
    {860.87,    -400.57	},
    {1125.59,   -703.77	},
    {1058.52,   -759.83	},
    {980.29,    -805.85	},
    {950.48,    -829.43	},
    {884.76,    -879.47	},
    {409.81,    -430.68	},
    {374.48,    -456.56	},
    {362.19,    -463.84	},
    {329.3,	    -485.63	},
    {219.97,    -374.13	},
    {194.08,    -416.5	},
    {185.61,    -432.34	},
    {159.59,    -449.51	},
    {147.88,    -459.81	},
    {119.33,    -474.73	},
    {90.54,	    -492.24	},
    {76.79,	    -507.22	},
    {45.29,	    -522.54	},
    {34.94,	    -539.87	},
    {-1.98,	    -558.5	}
};

/**
 * @brief Known scan data b captured at a specific pose, used for testing ICP alignment.
 */
static Point2D g_scan_b_points[OUTPUT_BUFFER] = {    
    {-377.89,	-619.31	},
    {-408.16,	-556.34	},
    {-434.21,	-520.06	},
    {-443.37,	-500.31	},
    {-479.09,	-477.27	},
    {-1022.23,	-826.41	},
    {-1048.31,	-806.67	},
    {-359.61,	-195.89	},
    {-371.73,	-172.97	},
    {-377.76,	-161.93	},
    {-391.33,	-139.66	},
    {-404.23,	-131.22	},
    {-419.05,	-107.03	},
    {-438.78,	-82.68	},
    {-455.42,	-71.5	},
    {-475.41,	-44.6	},
    {-492.45,	-32.14	},
    {-511,	    -0.98	},
    {-527.49,	32.7	},
    {-550.22,	50.1	},
    {-573.92,	90.42	},
    {-603.69,	112.06	},
    {-631.95,	168.41	},
    {-663.61,	194.41	},
    {-697.65,	254.14	},
    {-730.61,	322.19	},
    {-794.13,	377.98	},
    {-1921.39,	1072.59	},
    {-1758.73,	1098.31	},
    {-1619.98,	1089.48	},
    {-1353.59,	1102.86	},
    {-2658.8,	2461.81	},
    {-2377.6,	2644.94	},
    {-2155.62,	2898.25	},
    {-1960.16,	3016.58	},
    {-506.81,	830.59	},
    {-192.36,	466.93	},
    {-164.81,	484.22	},
    {-154.11,	498.73	},
    {-124.83,	515.6	},
    {-111.4,	531.96	},
    {-80.58,	554.68	},
    {-47.13,	580.59	},
    {-29.28,	599.29	},
    {7.98,	    622.45	},
    {26.86,	    651.95	},
    {114.33,	670.83	},
    {128.03,	633.7	},
    {157.32,	595.57	},
    {173.05,	585.98	},
    {225.16,	610.82	},
    {282.75,	643.08	},
    {305.58,	550.93	},
    {315.32,	525.1	},
    {337.18,	491.46	},
    {343.72,	469.04	},
    {370.11,	445.26	},
    {579.91,	578.33	},
    {810.29,	711	    },
    {782.44,	649.67	},
    {869.52,	632.11	},
    {5367.72,	2733.15	},
    {4101.95,	1627.44	},
    {3141.45,	1023.56	},
    {3502.39,	1023.96	},
    {2844.62,	641.23	},
    {3056.71,	598.49	},
    {3347.73,	345.4	},
    {3441.45,	132.4	},
    {2824.42,	-151.88	},
    {2871.73,	-337.51	},
    {2757.88,	-583.85	},
    {2837.62,	-802.8	},
    {4416.05,	-1377.82},
    {1271.64,	-529.57	},
    {739.53,	-425.09	},
    {951.3,	    -710.77	},
    {892.65,	-759.45	},
    {854.73,	-770.87	},
    {802.43,	-822.37	},
    {744.27,	-866.15	},
    {709.5,	    -888.47	},
    {653,	    -927.73	},
    {274.35,	-416.22	},
    {249.95,	-436.5	},
    {222.93,	-454.24	},
    {210.81,	-464.94	},
    {183.29,	-481.81	},
    {88.79,	    -360.73	},
    {49.11,	    -434.23	},
    {41,	    -439.59	},
    {11.79,	    -454.85	},
    {-0.64,	    -472	},
    {-32.12,	-487.94	},
    {-65.24,	-507.32	},
    {-83.78,	-529.92	},
    {-123.56,	-552.35	},
    {-152.15,	-585.04	},
    {-204.9	,	-624.23	},
    {-258.84,	-649.31	}
};

/**
 * @brief Known scan data c captured at a specific pose, used for testing ICP alignment.
 */
static Point2D g_scan_c_points[OUTPUT_BUFFER] = {
    {-422.47,	-638.66	},
    {-431.54,	-610.97	},
    {-462.4,	-573.25	},
    {-472.35,	-553.96	},
    {-501.49,	-517.33	},
    {-555.4,	-504.76	},
    {-1054.5,	-886.3	},
    {-1088.25,	-809.4	},
    {-428.11,	-208.66	},
    {-444.01,	-182.36	},
    {-456.11,	-172.1	},
    {-477.63,	-145.32	},
    {-494.03,	-135.41	},
    {-513.83,	-105.27	},
    {-530.34,	-93.36	},
    {-553.21,	-60.43	},
    {-573.85,	-27.41	},
    {-601.4,	-10.99	},
    {-620.12,	27.92	},
    {-644.91,	48.81	},
    {-675.54,	93.63	},
    {-707.14,	144.47	},
    {-782.86,	245.66	},
    {-833.79,	288.37	},
    {-2125.48,	967.94	},
    {-1703.91,	1061.49	},
    {-1451.34,	1080.69	},
    {-2885.16,	2278.31	},
    {-2524.49,	2575.96	},
    {-2300,	    2838.68	},
    {-266.5,	458.99	},
    {-255.77,	470.76	},
    {-226.65,	488.48	},
    {-214.7,	500.11	},
    {-186.1,	522.58	},
    {-174.2,	536.92	},
    {-143.3,	561.23	},
    {-110.2,	583.42	},
    {-96.48,	603.84	},
    {-59.68,	625.16	},
    {-39.49,	648.55	},
    {1.7,	    693.25	},
    {41.27,	    648.69	},
    {57.87,	    609.76	},
    {91.36,	    575.8	},
    {112.96,	601.23	},
    {159.02,	629.73	},
    {213.69,	659.49	},
    {201.44,	568.36	},
    {230.78,	540	},
    {239.59,	516.37	},
    {262.76,	484.88	},
    {269.27,	462.01	},
    {301.07,	449.79	},
    {727.81,	741.84	},
    {719.85,	645.68	},
    {801.64,	632.31	},
    {5310.27,	2769.87	},
    {3419.83,	1151.52	},
    {2722.83,	831.64	},
    {3017.33,	618.17	},
    {3246.47,	455.36	},
    {3343.94,	253.08	},
    {2703.7,	-40.56	},
    {2801.36,	-121.54	},
    {2738.18,	-553.2	},
    {2916.67,	-790.9	},
    {1266.09,	-471.4	},
    {1222.74,	-495.96	},
    {920.98,	-675.67	},
    {868.93,	-726.3	},
    {805.63,	-765.77	},
    {783.06,	-787.77	},
    {727.94,	-832.34	},
    {695.15,	-856.05	},
    {641.94,	-896.34	},
    {618,	    -917.84	},
    {212.76,	-424.69	},
    {202.71,	-435.65	},
    {175.61,	-450.48	},
    {164.36,	-463.73	},
    {137.09,	-480.58	},
    {121.76,	-564.52	},
    {96.93,	    -519.03	},
    {44.05,	    -362.84	},
    {12.89,	    -425.55	},
    {-13.62,	-441.79	},
    {-27.55,	-460.68	},
    {-59.27,	-477.33	},
    {-76.6,	    -500.42	},
    {-114.23,	-522.66	},
    {-141.06,	-552.27	},
    {-187.98,	-584.52	},
    {-245.23,	-624.05	},
    {-285.64,	-666.36	},
    {-345.56,	-684.19	},
    {-985.52,	-1818.64},
    {-405.25,	-657.67	},
    {-436.97,	-615.09	},
    {-446.43,	-591.42	}

};

/* ===== PASTE KNOWN POSES (inEKF output at capture time) =====
   Read from DEBUG_OUTPUT serial log: "Significant pose change detected..." */
static const Pose POSE_A = {-114.57f,  -1.66f, 2.884f, 0};
static const Pose POSE_B = {-167.12f,  -2.34f, 0.035f, 0};
static const Pose POSE_C = { -80.92f,  -2.67f, 0.049f, 0};

/* ---- swap these four lines to change which scan pair is tested ---- */
#define SCAN_SRC_PTS   g_scan_b_points
#define POSE_SRC       POSE_B
#define SCAN_TGT_PTS   g_scan_c_points
#define POSE_TGT       POSE_C



static PointCloud g_scan_a, g_scan_b;

static ICPResult g_icp_result_n = {};
static ICPResult g_icp_result_i = {};

static Pose g_initial_guess = {0.f, 0.f, 0.f, 0};

static float g_confidence_n = 0.f;
static float g_confidence_i = 0.f;


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Load test scans from a `Point2D` array into a `PointCloud` structure
 * 
 * @param source    Pointer to the array of `Point2D` structures
 * @param dest      Pointer to the `PointCloud` structure to populate
 */
void load_test_scans(
        const Point2D*    source,
              PointCloud* dest)
{
    for (uint32_t i = 0; i < OUTPUT_BUFFER; i++) {
        dest->points[i] = source[i];
    }
    dest->num_pts = OUTPUT_BUFFER;
}


/**
 * @brief Test that ICP returns a valid result structure
 * 
 * @note uses `TEST_ASSERT_TRUE()` to check the `valid` flag in the `ICPResult`
 *  structure, which should be set to true for a successful ICP alignment.
 */
void test_icp_returns_valid_result() {
    TEST_ASSERT_TRUE(g_icp_result_n.valid);
}


/**
 * @brief Test that ICP delta is close to known pose difference (from inEKF
 *  output at capture time)
 * 
 * @note uses `TEST_ASSERT_FLOAT_WITHIN()` for position and angle, with
 *  tolerances set based on expected sensor noise and capture conditions.
 */
void test_icp_delta_close_to_known_pose() {

    /* Expected transform: POSE_SRC body-frame -->  POSE_TGT body-frame
     *
     *   dtheta   = SRC.theta - TGT.theta
     *   [dx, dy] = R(-TGT.theta) * [SRC.x - TGT.x,  SRC.y - TGT.y]
     *
     * NOTE: this is SRC - TGT, not TGT - SRC.
     *   TGT - SRC  = how the robot moved in the GLOBAL frame.
     *   SRC - TGT  = the coordinate-frame transform ICP solves for:
     *                "rotate SRC's axes to align with TGT's axes".
     */
    float dx_g      = POSE_SRC.x     - POSE_TGT.x;
    float dy_g      = POSE_SRC.y     - POSE_TGT.y;
    float c         = cosf(-POSE_TGT.theta);
    float s         = sinf(-POSE_TGT.theta);
    float exp_dx    = c*dx_g - s*dy_g;
    float exp_dy    = s*dx_g + c*dy_g;
    float exp_dth   = normalize_angle(POSE_SRC.theta - POSE_TGT.theta);

    printf("pose_x: expected=%.2f, icp=%.2f\n",
           exp_dx,
           g_icp_result_n.dx);
    printf("pose_y: expected=%.2f, icp=%.2f\n",
           exp_dy,
           g_icp_result_n.dy);
    printf("pose_th: expected=%.3f, icp=%.3f\n",
           exp_dth,
           g_icp_result_n.dtheta);
    

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, exp_dx, g_icp_result_n.dx);
    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, exp_dy, g_icp_result_n.dy);
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD, exp_dth, g_icp_result_n.dtheta);

}

/**
 * @brief Test that improved ICP delta is close to known pose difference (from inEKF  output at capture time)
 * 
 * @note uses `TEST_ASSERT_FLOAT_WITHIN()` for position and angle, with
 *  tolerances set based on expected sensor noise and capture conditions.
 */
void test_improved_icp_delta_close_to_known_pose() {

    /* Expected transform: POSE_SRC body-frame -->  POSE_TGT body-frame
     *
     *   dtheta   = SRC.theta - TGT.theta
     *   [dx, dy] = R(-TGT.theta) * [SRC.x - TGT.x,  SRC.y - TGT.y]
     *
     * NOTE: this is SRC - TGT, not TGT - SRC.
     *   TGT - SRC  = how the robot moved in the GLOBAL frame.
     *   SRC - TGT  = the coordinate-frame transform ICP solves for:
     *                "rotate SRC's axes to align with TGT's axes".
     */
    float dx_g      = POSE_SRC.x     - POSE_TGT.x;
    float dy_g      = POSE_SRC.y     - POSE_TGT.y;
    float c         = cosf(-POSE_TGT.theta);
    float s         = sinf(-POSE_TGT.theta);
    float exp_dx    = c*dx_g - s*dy_g;
    float exp_dy    = s*dx_g + c*dy_g;
    float exp_dth   = normalize_angle(POSE_SRC.theta - POSE_TGT.theta);

    printf("pose_x: expected=%.2f, icp=%.2f\n",
           exp_dx,
           g_icp_result_i.dx);
    printf("pose_y: expected=%.2f, icp=%.2f\n",
           exp_dy,
           g_icp_result_i.dy);
    printf("pose_th: expected=%.3f, icp=%.3f\n",
           exp_dth,
           g_icp_result_i.dtheta);
    

    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, exp_dx, g_icp_result_i.dx);
    TEST_ASSERT_FLOAT_WITHIN(POSITION_TOLERANCE_MM, exp_dy, g_icp_result_i.dy);
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD, exp_dth, g_icp_result_i.dtheta);

}

/**
 * @brief Test that ICP confidence is above a reasonable floor value
 * 
 * @note uses `TEST_ASSERT_GREATER_OR_EQUAL_FLOAT()` to check that the computed
 *  confidence metric for the ICP result is above a defined floor, defined
 *  as `CONFIDENCE_FLOOR`.
 */
void test_icp_confidence_above_floor() {

    // prints the resulting confidence for reference
    printf("  ICP confidence = %.3f\n", g_confidence_n);

    // required confidence floor for test to pass
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(CONFIDENCE_FLOOR,
                                       g_confidence_n);
}

/**
 * @brief Test that improved ICP confidence is also above a reasonable floor value
 * 
 * @note uses `TEST_ASSERT_GREATER_OR_EQUAL_FLOAT()` to check that the computed
 *  confidence metric for the ICP result is above a defined floor, defined
 *  as `CONFIDENCE_FLOOR`.
 */
void test_improved_icp_confidence_above_floor() {

    // prints the resulting confidence for reference
    printf("  ICP confidence = %.3f\n", g_confidence_i);

    // required confidence floor for test to pass
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(CONFIDENCE_FLOOR,
                                       g_confidence_n);
}

void test_icp_improved_time_above_unimproved() {

}


// ----------------------------------------------------------------------------
//
//  TEST HARNESS
//  
// ----------------------------------------------------------------------------

/**
 * @brief used to initialize test data once
 */
void init_test_data() {

    struct timespec start_time, end_time;

    // load scan data — pair selected by SCAN_SRC_PTS / SCAN_TGT_PTS above
    load_test_scans(SCAN_SRC_PTS, &g_scan_a);
    load_test_scans(SCAN_TGT_PTS, &g_scan_b);

    g_initial_guess = (Pose){
            // achieved confidence of 0.664 in 6 iterations
            // 0.f,
            // 0.f,
            // 0.f,
            // achieved confidence of 0.666 in 7 iterations
            0.5*(POSE_SRC.x - POSE_TGT.x),
            0.5*(POSE_SRC.y - POSE_TGT.y),
            0.5*normalize_angle(POSE_SRC.theta - POSE_TGT.theta),
            0};

    // Perform ICP and compute confidence for unimproved algorithm (ICP_2D)
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    slam_perform_icp(
            &g_scan_a,
            &g_scan_b,
            &g_initial_guess,
            &g_icp_result_n);

    clock_gettime(CLOCK_MONOTONIC, &end_time);

    long time_n =  (end_time.tv_sec - start_time.tv_sec) * 1000000000L
                 + (end_time.tv_nsec - start_time.tv_nsec);
    printf("Unimproved ICP time: %ld ns\n", time_n);

    g_confidence_n = slam_compute_icp_confidence(
            &g_scan_a,
            &g_scan_b,
            &g_icp_result_n);


    // Perform ICP and compute confidence for improved algorithm (ICP_2D_i)
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    slam_perform_icp_i(
            &g_scan_a,
            &g_scan_b,
            &g_initial_guess,
            &g_icp_result_i);

    clock_gettime(CLOCK_MONOTONIC, &end_time);

    long time_i =  (end_time.tv_sec - start_time.tv_sec) * 1000000000L
                 + (end_time.tv_nsec - start_time.tv_nsec);
    printf("Improved ICP time: %ld ns\n", time_i);

    g_confidence_i = slam_compute_icp_confidence(
            &g_scan_a,
            &g_scan_b,
            &g_icp_result_i);
}



/**
 * @brief Set the Up object. This is meant to run BEFORE each test
 * 
 * @note  function is left empty because shared states are initialized once only in `init_test_data()`.
 */
void setUp() {}

/**
 * @brief Tear down the test environment. This is meant to run AFTER each test
 * 
 * @note  function is left empty because shared states are initialized once only in `init_test_data()`.
 */
void tearDown() {}

int main(void) {
    UNITY_BEGIN();

    init_test_data();  // Run ICP once before all tests

    printf("\n\n");

    RUN_TEST(test_icp_returns_valid_result);
    RUN_TEST(test_icp_delta_close_to_known_pose);
    RUN_TEST(test_icp_confidence_above_floor);
    
    printf("\n\n");
    
    // RUN_TEST(test_icp_returns_valid_result);
    RUN_TEST(test_improved_icp_delta_close_to_known_pose);
    RUN_TEST(test_improved_icp_confidence_above_floor);


    return UNITY_END();
}