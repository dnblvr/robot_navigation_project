/**
 * @file test_algorithms.cpp
 * @brief Native PC simulation for testing algorithms with breakpoints
 * 
 * This file allows you to test your algorithms (Madgwick, quaternions, etc.)
 * on your PC with full debugging support - breakpoints, stepping, etc.
 * 
 * To run:
 *   pio run -e native_sim
 * 
 * To debug in VSCode:
 *   Set breakpoints, then press F5 and select "PIO Debug (Native)"
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Mock Arduino for native environment
#define NATIVE_SIM
#include "Arduino_mock.h"
MockSerial Serial;  // Define the mock serial instance

// Include your algorithm headers
#include "helper_3dmath.h"
#include "MadgwickFilter.h"

// =============================================================================
// Debug Helper Macros - Makes printf debugging easier!
// =============================================================================

#define DEBUG_PRINT(msg)            printf("[DEBUG] %s\n", msg)
#define DEBUG_VAR(name, value)      printf("[DEBUG] %s = %d\n", name, value)
#define DEBUG_FLOAT(name, value)    printf("[DEBUG] %s = %.6f\n", name, value)
#define DEBUG_VEC3(name, vec)       printf("[DEBUG] %s = (x:%.3f, y:%.3f, z:%.3f)\n", name, (vec).x, (vec).y, (vec).z)
#define DEBUG_VEC3_INT(name, vec)   printf("[DEBUG] %s = (x:%d, y:%d, z:%d)\n", name, (vec).x, (vec).y, (vec).z)
#define DEBUG_QUAT(name, q)         printf("[DEBUG] %s = (w:%.4f, x:%.4f, y:%.4f, z:%.4f)\n", name, (q).w, (q).x, (q).y, (q).z)
#define DEBUG_LINE()                printf("----------------------------------------\n")
#define DEBUG_SECTION(title)        printf("\n>>> %s <<<\n", title)

// =============================================================================
// Test Data Generator
// =============================================================================

/**
 * @brief Generate simulated IMU data for testing
 */
void generateMockIMUData(VectorInt16* accel, VectorInt16* gyro, VectorInt16* mag) {
    // Simulate sensor readings (hovering, slight tilt)
    accel->x = 100;    // Small X tilt
    accel->y = -50;    // Small Y tilt  
    accel->z = 16384;  // ~1g in Z (pointing up)
    
    gyro->x = 10;      // Small rotation
    gyro->y = -5;
    gyro->z = 2;
    
    mag->x = 200;      // North-ish
    mag->y = 50;
    mag->z = -300;
}

// =============================================================================
// Test Functions
// =============================================================================

void test_quaternion_operations() {
    printf("\n=== Testing Quaternion Operations ===\n");
    
    // Create quaternions
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    Quaternion q2(0.707f, 0.0f, 0.707f, 0.0f);  // 90° rotation around Y
    
    printf("q1: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
           q1.w, q1.x, q1.y, q1.z);
    printf("q2: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
           q2.w, q2.x, q2.y, q2.z);
    
    // Multiply quaternions - SET BREAKPOINT HERE
    Quaternion q3 = q1.getProduct(q2);
    printf("q3 (q1*q2): w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
           q3.w, q3.x, q3.y, q3.z);
    
    // Normalize
    q3.normalize();
    printf("q3 normalized: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
           q3.w, q3.x, q3.y, q3.z);
    
    // Get magnitude
    float mag = q3.getMagnitude();
    printf("q3 magnitude: %.6f (should be ~1.0)\n",
           mag);
    
    printf("✓ Quaternion operations test complete\n");
}

void test_vector_operations() {
    printf("\n=== Testing Vector Operations ===\n");
    
    VectorInt16 v1(100, 200, 300);
    VectorInt16 v2(-50, 100, -150);
    
    printf("v1: x=%d, y=%d, z=%d\n", v1.x, v1.y, v1.z);
    printf("v2: x=%d, y=%d, z=%d\n", v2.x, v2.y, v2.z);
    
    // Magnitude - SET BREAKPOINT HERE
    float mag = v1.getMagnitude();
    printf("v1 magnitude: %.2f\n", mag);
    
    // Normalize to VectorFloat
    VectorFloat vf;
    vf.x = v1.x / mag;
    vf.y = v1.y / mag;
    vf.z = v1.z / mag;
    printf("v1 normalized: x=%.3f, y=%.3f, z=%.3f\n", vf.x, vf.y, vf.z);
    
    printf("✓ Vector operations test complete\n");
}

void test_gravity_extraction() {
    printf("\n=== Testing Gravity Extraction ===\n");
    
    // Create an orientation quaternion (slightly tilted)
    Quaternion q(0.966f, 0.259f, 0.0f, 0.0f);  // ~30° tilt
    VectorFloat gravity;
    
    // Extract gravity from orientation - SET BREAKPOINT HERE
    dmpGetGravity(&q, &gravity);
    
    printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", q.w, q.x, q.y, q.z);
    printf("Gravity vector: x=%.3f, y=%.3f, z=%.3f\n", gravity.x, gravity.y, gravity.z);
    
    // Magnitude should be ~1.0
    float mag = sqrtf(gravity.x*gravity.x + gravity.y*gravity.y + gravity.z*gravity.z);
    printf("Gravity magnitude: %.6f (should be ~1.0)\n", mag);
    
    printf("✓ Gravity extraction test complete\n");
}

void test_linear_acceleration() {
    printf("\n=== Testing Linear Acceleration Calculation ===\n");
    
    // Raw accelerometer reading (with gravity)
    VectorInt16 rawAccel(500, -200, 16384);  // Some motion + gravity
    
    // Gravity from orientation
    VectorFloat gravity;
    gravity.x = 0.1f;
    gravity.y = -0.05f;
    gravity.z = 1.0f;  // Pointing mostly up
    
    // Remove gravity to get linear acceleration - SET BREAKPOINT HERE
    VectorInt16 linearAccel;
    dmpGetLinearAccel(&rawAccel, &gravity, &linearAccel);
    
    printf("Raw accel: x=%d, y=%d, z=%d\n", rawAccel.x, rawAccel.y, rawAccel.z);
    printf("Gravity: x=%.3f, y=%.3f, z=%.3f\n", gravity.x, gravity.y, gravity.z);
    printf("Linear accel: x=%d, y=%d, z=%d\n", linearAccel.x, linearAccel.y, linearAccel.z);
    
    printf("✓ Linear acceleration test complete\n");
}

void test_madgwick_filter() {
    printf("\n=== Testing Madgwick Filter ===\n");
    
    DEBUG_SECTION("Madgwick Filter Initialization");
    
    // Create Madgwick filter instance
    Madgwick_Filter MF(16, 14, 17.0f, 0.01f);  // 10ms sample time
    DEBUG_PRINT("Filter created");
    
    // Generate mock sensor data
    VectorInt16 accel, gyro, mag;
    generateMockIMUData(&accel, &gyro, &mag);
    
    printf("\nMock sensor data:\n");
    DEBUG_VEC3_INT("  Accel", accel);
    DEBUG_VEC3_INT("  Gyro", gyro);
    DEBUG_VEC3_INT("  Mag", mag);
    
    DEBUG_SECTION("Running Filter Iterations");
    
    // Run multiple filter iterations with detailed tracing
    printf("\nRunning filter for 10 iterations...\n");
    for (int i = 0; i < 10; i++) {
        DEBUG_LINE();
        printf("ITERATION %d:\n", i);
        
        VectorFloat magFloat;
        magFloat.x = (float)mag.x;
        magFloat.y = (float)mag.y;
        magFloat.z = (float)mag.z;
        
        DEBUG_VEC3("  Input mag (float)", magFloat);
        DEBUG_QUAT("  Before filter", MF.wOriN);
        
        // This is where you can trace through the algorithm
        // Add printf statements inside MadgwickFilter.cpp to see internals
        MF.DMP_Main(&accel, &magFloat, &MF.wOriN, gyro.getMagnitude());
        
        DEBUG_QUAT("  After filter", MF.wOriN);
        
        // Check for NaN
        if (isnan(MF.wOriN.w) || isnan(MF.wOriN.x) || isnan(MF.wOriN.y) || isnan(MF.wOriN.z)) {
            printf("  ⚠️  WARNING: NaN detected at iteration %d!\n", i);
            printf("  This usually means:\n");
            printf("    - Division by zero\n");
            printf("    - Invalid input data\n");
            printf("    - Uninitialized quaternion\n");
            break;
        }
    }
    
    printf("\n✓ Madgwick filter test complete\n");
}

// =============================================================================
// Main Test Runner
// =============================================================================

int main(int argc, char** argv) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   ALGORITHM SIMULATION & DEBUG TEST SUITE                ║\n");
    printf("║   Set breakpoints and step through with F10/F11!         ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    
    // Run all tests
    test_quaternion_operations();
    test_vector_operations();
    test_gravity_extraction();
    test_linear_acceleration();
    test_madgwick_filter();
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   ALL TESTS COMPLETE!                                     ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    return 0;
}
