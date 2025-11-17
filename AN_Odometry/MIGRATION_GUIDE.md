# Migration Guide: Old vs New SLAM Implementation

## Overview

This document helps migrate from the old graph-based SLAM implementation to the new functional SLAM implementation.

## Conceptual Differences

| Aspect | Old Implementation | New Implementation |
|--------|-------------------|-------------------|
| **Data Structure** | Linked list of graph nodes | Fixed-size arrays |
| **Memory Model** | Dynamic allocation | Pre-allocated pools |
| **API Style** | Graph manipulation | Functional calls |
| **Constraints** | Stored in edge structures | Stored in sparse matrix |
| **Landmarks** | Separate landmark nodes | Currently pose-only (extensible) |

## Data Structure Comparison

### Old: Graph Nodes

```c
// Old implementation
typedef struct GraphNode {
    PoseState pose;
    Observation *observations;
    struct GraphNode *next;
} GraphNode;

typedef struct Edge {
    GraphNode *from;
    GraphNode *to;
    float z_ij[3];  // measurement
} Edge;
```

### New: Fixed Arrays

```c
// New implementation
typedef struct {
    Pose pose_pool[MAX_POSES];
    PointCloud scan_pool[MAX_POSES];
    int current_pose_count;
    SparseMatrix H;  // Constraints stored here
    float b[STATE_SIZE];
    float state[STATE_SIZE];
} SLAMOptimizer;
```

## Function Migration Table

### Initialization

| Old | New |
|-----|-----|
| N/A (manual node creation) | `slam_initialize(SLAMOptimizer*)` |

### Adding Data

| Old | New |
|-----|-----|
| Create `GraphNode`, link manually | `slam_add_pose(optimizer, pose, scan)` |
| Store observations in node | Scans stored in `scan_pool` |
| Create `Edge` structures | Constraints added via functions |

### Adding Constraints

| Old | New |
|-----|-----|
| Manual edge creation | `slam_add_odometry_constraint(...)` |
| N/A | `slam_add_icp_constraint(...)` |
| Manual loop closure | `slam_detect_loop_closure(...)` |

### Optimization

| Old | New |
|-----|-----|
| `gauss_newton_slam(head, landmarks, ...)` | `slam_optimize_gauss_newton(optimizer, iters)` |
| `build_state_vector(...)` | Done internally |
| `linearize_constraints(...)` | Done internally |
| `solve_linear_system(...)` | Done internally |
| `write_state_to_graph(...)` | Done internally |

### Accessing Results

| Old | New |
|-----|-----|
| Traverse linked list | `slam_get_pose(optimizer, id, out_pose)` |
| Access node->pose | `slam_get_current_pose(optimizer, out_pose)` |
| N/A | `slam_get_scan(optimizer, id, out_scan)` |

## Code Examples

### Old Way: Creating a Graph

```c
// Old implementation
GraphNode *head = NULL;
GraphNode *current = NULL;

// Add first pose
GraphNode *node1 = (GraphNode*)malloc(sizeof(GraphNode));
node1->pose.x = 0.0f;
node1->pose.y = 0.0f;
node1->pose.theta = 0.0f;
node1->next = NULL;
head = node1;
current = node1;

// Add second pose
GraphNode *node2 = (GraphNode*)malloc(sizeof(GraphNode));
node2->pose.x = 1.0f;
node2->pose.y = 0.0f;
node2->pose.theta = 0.0f;
node2->next = NULL;
current->next = node2;
current = node2;

// Add observations
node1->num_observations = 1;
node1->observations = malloc(sizeof(Observation));
node1->observations[0].landmark = landmark_ptr;
node1->observations[0].range = 5.0f;
node1->observations[0].bearing = 0.5f;

// Optimize
float state[STATE_SIZE];
build_state_vector(head, landmarks, state, num_poses, num_landmarks);
gauss_newton_slam(head, landmarks, num_poses, num_landmarks, 10, 1e-4f);

// Access results - traverse list
current = head;
while (current != NULL) {
    printf("Pose: (%.2f, %.2f, %.2f)\n", 
           current->pose.x, current->pose.y, current->pose.theta);
    current = current->next;
}

// Cleanup - manual traversal and freeing
```

### New Way: Functional Approach

```c
// New implementation
SLAMOptimizer optimizer;
slam_initialize(&optimizer);

// Add first pose
Pose pose1 = {0.0f, 0.0f, 0.0f, 0};
PointCloud scan1 = {/* scan data */};
slam_add_pose(&optimizer, &pose1, &scan1);

// Add second pose
Pose pose2 = {1.0f, 0.0f, 0.0f, 0};
PointCloud scan2 = {/* scan data */};
slam_add_pose(&optimizer, &pose2, &scan2);

// Add odometry constraint
slam_add_odometry_constraint(&optimizer, 0, 1, 1.0f, 0.0f, 0.0f, 100.0f);

// Add ICP constraint
ICPResult icp_result;
slam_perform_icp(&scan1, &scan2, &pose_estimate, &icp_result);
slam_add_icp_constraint(&optimizer, 0, 1, &icp_result, 50.0f);

// Optimize
slam_optimize_gauss_newton(&optimizer, 10);

// Access results - direct indexing
for (int i = 0; i < optimizer.current_pose_count; i++) {
    Pose result;
    slam_get_pose(&optimizer, i, &result);
    printf("Pose %d: (%.2f, %.2f, %.2f)\n", i, result.x, result.y, result.theta);
}

// Cleanup - single function
slam_cleanup(&optimizer);
```

## Key Advantages of New Implementation

1. **Simpler API**: No manual list management
2. **Better memory control**: Pre-allocated, predictable
3. **Faster access**: Direct array indexing vs list traversal
4. **ICP integration**: Built-in scan matching
5. **Loop closure**: Automatic detection
6. **Type safety**: Strongly typed structures vs void pointers

## Limitations of New Implementation

1. **Fixed capacity**: Limited to MAX_POSES (configurable)
2. **No landmarks**: Currently only pose graph (can be extended)
3. **Dense solver**: Temporary high memory during optimization
4. **Less flexible**: Can't have arbitrary graph topology

## Feature Parity Matrix

| Feature | Old | New | Notes |
|---------|-----|-----|-------|
| Pose graph | ✓ | ✓ | |
| Landmarks | ✓ | ✗ | Can be added if needed |
| Odometry constraints | ✓ | ✓ | |
| Scan matching | ✗ | ✓ | Via ICP |
| Loop closure | Manual | ✓ | Automatic detection |
| Gauss-Newton | ✓ | ✓ | |
| Cholesky solver | ✓ | ✓ | |
| Sparse matrices | Dense | ✓ | COO format |
| Dynamic graph | ✓ | ✗ | Fixed size |
| Memory efficient | ✗ | ✓ | Pre-allocated |

## Migration Checklist

- [ ] Replace `graphslam.h` include path if needed
- [ ] Remove old graph node creation code
- [ ] Initialize `SLAMOptimizer` with `slam_initialize()`
- [ ] Replace node creation with `slam_add_pose()`
- [ ] Replace manual edges with constraint functions
- [ ] Update optimization call to `slam_optimize_gauss_newton()`
- [ ] Change result access from list traversal to `slam_get_pose()`
- [ ] Add `slam_cleanup()` at shutdown
- [ ] Test with small dataset first
- [ ] Monitor memory usage

## Common Migration Patterns

### Pattern 1: Adding Poses with Odometry

**Old**:
```c
GraphNode *prev = current_node;
GraphNode *curr = create_new_node(x, y, theta);
link_nodes(prev, curr);
add_odometry_edge(prev, curr, dx, dy, dtheta);
```

**New**:
```c
Pose curr_pose = {x, y, theta, timestamp};
PointCloud curr_scan = get_scan();
slam_add_pose(&optimizer, &curr_pose, &curr_scan);
slam_add_odometry_constraint(&optimizer, prev_id, curr_id, dx, dy, dtheta, confidence);
```

### Pattern 2: Loop Closure

**Old**:
```c
// Manual search and matching
for (GraphNode *candidate = head; candidate != NULL; candidate = candidate->next) {
    if (distance(current, candidate) < threshold) {
        // Manual ICP or matching
        // Manual edge creation
    }
}
```

**New**:
```c
// Automatic detection
if (slam_detect_loop_closure(&optimizer, current_pose_id)) {
    printf("Loop closure detected and added!\n");
}
```

### Pattern 3: Optimization

**Old**:
```c
float state[STATE_SIZE];
float b[STATE_SIZE];
float H[STATE_SIZE][STATE_SIZE];

build_state_vector(head, landmarks, state, num_poses, num_landmarks);
linearize_constraints(head, landmarks, state, b, H, num_poses, num_landmarks);
solve_linear_system(H, b, dx, state_size);
update_state_vector(state, dx, state_size);
write_state_to_graph(head, landmarks, state, num_poses, num_landmarks);
```

**New**:
```c
// All done in one call
slam_optimize_gauss_newton(&optimizer, max_iterations);
```

## Debugging Tips

1. **Check pose count**: `optimizer.current_pose_count` should increase
2. **Verify sparse matrix**: `optimizer.H.nnz` shows constraint count
3. **Monitor convergence**: Add printf in optimization loop
4. **Check ICP results**: Print `icp_result.confidence` values
5. **Memory tracking**: Use your existing memory functions

## Performance Comparison

| Metric | Old | New |
|--------|-----|-----|
| Pose access time | O(n) | O(1) |
| Memory overhead | High (pointers) | Low (arrays) |
| Optimization setup | Manual | Automatic |
| ICP integration | N/A | Built-in |
| Loop closure | Manual | Automatic |

## Questions & Troubleshooting

### Q: Can I still use landmarks?
**A**: The current implementation focuses on pose graphs. Landmark support can be added by extending the data structures and adding landmark constraint functions (similar to the old `evaluate_error_pose_landmark`).

### Q: What if I exceed MAX_POSES?
**A**: `slam_add_pose()` returns false when full. Consider:
- Increasing `MAX_POSES` in header
- Implementing a sliding window
- Marginalizing old poses

### Q: Why is optimization slow?
**A**: The sparse-to-dense conversion for Cholesky is expensive. Future improvement: implement true sparse Cholesky solver.

### Q: How do I save the map?
**A**: Iterate through poses with `slam_get_pose()` and write to file/UART:

```c
for (int i = 0; i < optimizer.current_pose_count; i++) {
    Pose p;
    slam_get_pose(&optimizer, i, &p);
    fprintf(file, "%.3f,%.3f,%.3f\n", p.x, p.y, p.theta);
}
```

## Additional Resources

- `GRAPHSLAM_README.md` - Complete API documentation
- `IMPLEMENTATION_SUMMARY.md` - Overview of new implementation
- `main.cpp` (Teensy) - Reference C++ implementation
- `chat_responses - condensed.md` - Design rationale
