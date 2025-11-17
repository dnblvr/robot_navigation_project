# Revised C-Based Graph SLAM Implementation

## Overview

This is a revised C implementation of Graph SLAM for the MSP432 microcontroller, based on the Teensy 4.1 C++ implementation found in `main.cpp`. The implementation uses a functional approach (no graph data structures) and integrates with existing ICP and matrix utilities.

## Key Features

### Memory-Efficient Design
- **Sparse matrix representation** using COO (Coordinate) format
- **Fixed-size arrays** for pose and scan storage (100 poses, 90 points/scan)
- **Pre-allocated buffers** to avoid dynamic allocation during optimization
- **Total estimated memory**: ~150KB for full capacity

### Integrated ICP Support
- Reuses existing `icp_2d.c` implementation
- Automatic confidence computation based on match quality
- Loop closure detection using spatial proximity + ICP verification

### Pose Graph Optimization
- **Gauss-Newton solver** with Cholesky decomposition
- **Odometry constraints** from wheel encoders
- **ICP constraints** from LiDAR scan matching
- **Loop closure constraints** for drift correction

## File Structure

### Header File
**Location**: `AN_Odometry/inc/graphslam.h`

**Key Components**:
- `Pose`: Pose representation (x, y, theta, timestamp)
- `PointCloud`: LiDAR scan data structure
- `ICPResult`: ICP alignment results with confidence
- `SparseMatrix`: Memory-efficient matrix storage
- `SLAMOptimizer`: Main SLAM state container

### Source File
**Location**: `AN_Odometry/src/graphslam.c`

**Note**: This is the complete implementation - no renaming needed!

## API Usage

### Initialization

```c
#include "graphslam.h"

SLAMOptimizer optimizer;

void setup() {
    slam_initialize(&optimizer);
}
```

### Adding Poses and Scans

```c
// Create a pose
Pose current_pose;
current_pose.x = 1.5f;
current_pose.y = 2.0f;
current_pose.theta = 0.785f;  // 45 degrees
current_pose.timestamp = get_current_time_ms();

// Create a point cloud from LiDAR data
PointCloud scan;
scan.num_points = num_measurements;
for (int i = 0; i < num_measurements; i++) {
    scan.points[i].x = lidar_x[i];
    scan.points[i].y = lidar_y[i];
}

// Add to SLAM
slam_add_pose(&optimizer, &current_pose, &scan);
```

### Adding Constraints

#### Odometry Constraints
```c
// Add odometry constraint between consecutive poses
slam_add_odometry_constraint(
    &optimizer,
    pose_id_1,      // Previous pose index
    pose_id_2,      // Current pose index
    dx,             // Relative x displacement
    dy,             // Relative y displacement
    dtheta,         // Relative angle change
    100.0f          // Confidence (higher = more certain)
);
```

#### ICP Constraints
```c
// Perform ICP between two scans
PointCloud *scan1, *scan2;
slam_get_scan(&optimizer, pose_id_1, scan1);
slam_get_scan(&optimizer, pose_id_2, scan2);

Pose initial_guess;
initial_guess.x = estimated_dx;
initial_guess.y = estimated_dy;
initial_guess.theta = estimated_dtheta;

ICPResult icp_result;
slam_perform_icp(scan1, scan2, &initial_guess, &icp_result);

// Compute confidence
float confidence = slam_compute_icp_confidence(scan1, scan2, &icp_result);
icp_result.confidence = confidence;

// Add constraint if confident enough
if (confidence > 0.8f) {
    slam_add_icp_constraint(
        &optimizer,
        pose_id_1,
        pose_id_2,
        &icp_result,
        50.0f  // Base confidence for ICP
    );
}
```

#### Loop Closure Detection
```c
// Automatically detect and add loop closures
bool loop_found = slam_detect_loop_closure(&optimizer, current_pose_id);

if (loop_found) {
    printf("Loop closure detected!\n");
}
```

### Optimization

```c
// Optimize the pose graph
slam_optimize_gauss_newton(&optimizer, 5);  // 5 Gauss-Newton iterations

// Get optimized poses
Pose optimized_pose;
slam_get_pose(&optimizer, pose_id, &optimized_pose);
```

### Cleanup

```c
void shutdown() {
    slam_cleanup(&optimizer);
}
```

## Integration Example

Here's a complete example similar to the Teensy implementation:

```c
#include "graphslam.h"
#include "RPLiDAR_C1.h"
#include "Tachometer.h"

SLAMOptimizer slam_optimizer;
int pose_counter = 0;

void process_lidar_scan() {
    // Get current odometry
    float dx = get_odometry_dx();
    float dy = get_odometry_dy();
    float dtheta = get_odometry_dtheta();
    
    // Update pose estimate
    Pose current_pose;
    if (pose_counter == 0) {
        current_pose.x = 0.0f;
        current_pose.y = 0.0f;
        current_pose.theta = 0.0f;
    } else {
        Pose prev_pose;
        slam_get_current_pose(&slam_optimizer, &prev_pose);
        compose_poses(&prev_pose, &(Pose){dx, dy, dtheta}, &current_pose);
    }
    current_pose.timestamp = get_time_ms();
    
    // Get LiDAR scan
    PointCloud scan;
    scan.num_points = get_lidar_points(scan.points, MAX_POINTS_PER_SCAN);
    
    // Add to SLAM
    slam_add_pose(&slam_optimizer, &current_pose, &scan);
    
    // Add odometry constraint
    if (pose_counter > 0) {
        slam_add_odometry_constraint(
            &slam_optimizer,
            pose_counter - 1,
            pose_counter,
            dx, dy, dtheta,
            100.0f
        );
    }
    
    // ICP constraint with previous pose
    if (pose_counter > 0) {
        PointCloud prev_scan;
        slam_get_scan(&slam_optimizer, pose_counter - 1, &prev_scan);
        
        Pose initial_guess = {dx, dy, dtheta};
        ICPResult icp_result;
        slam_perform_icp(&scan, &prev_scan, &initial_guess, &icp_result);
        
        float confidence = slam_compute_icp_confidence(&scan, &prev_scan, &icp_result);
        icp_result.confidence = confidence;
        
        if (confidence > 0.8f) {
            slam_add_icp_constraint(
                &slam_optimizer,
                pose_counter - 1,
                pose_counter,
                &icp_result,
                50.0f
            );
        }
    }
    
    // Check for loop closures
    if (pose_counter >= MIN_TEMPORAL_GAP) {
        slam_detect_loop_closure(&slam_optimizer, pose_counter);
    }
    
    // Optimize periodically
    if (pose_counter % OPTIMIZE_INTERVAL == 0 && pose_counter > 0) {
        slam_optimize_gauss_newton(&slam_optimizer, 5);
        
        // Get optimized current pose
        slam_get_current_pose(&slam_optimizer, &current_pose);
        printf("Optimized pose %d: (%.2f, %.2f, %.2f)\n",
               pose_counter, current_pose.x, current_pose.y, current_pose.theta);
    }
    
    pose_counter++;
}

int main(void) {
    // Initialize SLAM
    slam_initialize(&slam_optimizer);
    
    // Main loop
    while (1) {
        if (lidar_scan_ready()) {
            process_lidar_scan();
        }
        
        // Other tasks...
    }
    
    // Cleanup
    slam_cleanup(&slam_optimizer);
    return 0;
}
```

## Configuration Parameters

Located in `graphslam.h`:

```c
#define MAX_POSES               100     // Maximum poses to store
#define MAX_POINTS_PER_SCAN     90      // Points per LiDAR scan
#define LOOP_DISTANCE_THRESHOLD 2.0f    // Loop closure distance (meters)
#define ICP_CONFIDENCE_THRESHOLD 0.8f   // Minimum ICP confidence
#define MIN_TEMPORAL_GAP        20      // Minimum pose gap for loop closure
#define OPTIMIZE_INTERVAL       10      // Optimize every N poses
#define MAX_GAUSS_NEWTON_ITERS  5       // Max optimization iterations
#define CONVERGENCE_TOLERANCE   1e-4f   // Convergence threshold
```

## Dependencies

This implementation requires the following existing modules:

1. **icp_2d.h/c** - ICP algorithm implementation
2. **matrices.h/c** - Matrix operations
3. **cholesky_decomposition.h/c** - Linear system solver
4. **coordinate_transform.h/c** - Coordinate transformations

All dependencies are already present in the `AN_Odometry/inc` and `AN_Odometry/src` directories.

## Memory Usage Estimates

| Component | Size | Quantity | Total |
|-----------|------|----------|-------|
| Pose pool | 16 bytes | 100 | 1.6 KB |
| Scan pool | ~720 bytes | 100 | 72 KB |
| State vector | 4 bytes | 300 | 1.2 KB |
| Sparse H matrix | 12 bytes/entry | 1200 | 14.4 KB |
| Total | | | **~89 KB** |

Additional temporary memory during optimization:
- Dense H matrix: ~360 KB (freed after optimization)
- Cholesky factors: ~360 KB (freed after optimization)

**Note**: The dense matrix conversion is a memory bottleneck. For systems with <1MB RAM, consider optimizing in smaller batches or implementing a true sparse Cholesky solver.

## Differences from Original Implementation

1. **No graph nodes**: Uses arrays instead of linked lists
2. **C-style API**: Functions take `SLAMOptimizer*` instead of class methods
3. **Sparse matrix**: Custom COO format instead of Eigen::SparseMatrix
4. **Integrated ICP**: Uses existing `icp_2d.c` directly
5. **Fixed memory**: All allocations at initialization (except optimization temps)

## Future Improvements

1. **Sparse Cholesky**: Implement sparse solver to reduce memory during optimization
2. **Sliding window**: Keep only recent N poses to bound memory
3. **Incremental optimization**: Update only affected poses
4. **UART serialization**: Functions to send/receive SLAM state
5. **Map saving**: Export optimized map to SD card

## Testing Recommendations

1. **Unit tests**: Test individual functions with known inputs
2. **Small graphs**: Start with 5-10 poses to verify correctness
3. **Memory profiling**: Monitor RAM usage during operation
4. **Ground truth**: Compare with known trajectories
5. **Loop closure**: Test with circular paths

## References

- Original Teensy implementation: `teensy_coprocessor/src/main.cpp`
- ICP algorithm: `AN_Odometry/src/icp_2d.c`
- Design notes: `notes - Future Projects/Autonomous_Navigator/chat_responses - condensed.md`
