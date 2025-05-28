
#include "matrices.h"
#include "coordinate_transform.h"
#include "ransac.h"
#include "graphslam.h"
#include "rrt_star.h"

// #include "Motor.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <time.h>




typedef struct {
    float x_min, x_max, y_min, y_max;
} Obstacle_GS;

float randn(float mean, float stddev) {
    // Box-Muller transform for Gaussian noise
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return mean + stddev * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

int is_in_Obstacle(float x, float y, Obstacle_GS *obs) {
    return (x >= obs->x_min && x <= obs->x_max && y >= obs->y_min && y <= obs->y_max);
}

void example_rectangle_room_with_inner_Obstacle(void) {
    srand((unsigned int)time(NULL));

    // --- Room and Obstacle Setup ---
    float outer_x_min = 0.0f, outer_x_max = 20.0f;
    float outer_y_min = 0.0f, outer_y_max = 20.0f;
    Obstacle_GS inner_room = {8.0f, 12.0f, 8.0f, 12.0f};

    // --- Landmarks: corners of outer and inner room ---
    LandmarkMeasurement* landmarks[MAX_LANDMARKS] = {0};
    int num_landmarks = 0;

    // Outer room corners
    float outer_corners[4][2] = {
        {outer_x_min, outer_y_min},
        {outer_x_max, outer_y_min},
        {outer_x_max, outer_y_max},
        {outer_x_min, outer_y_max}
    };
    
    for (int i = 0; i < 4; ++i) {
        landmarks[num_landmarks] = malloc(sizeof(LandmarkMeasurement));
        landmarks[num_landmarks]->id = num_landmarks;
        landmarks[num_landmarks]->x = outer_corners[i][0];
        landmarks[num_landmarks]->y = outer_corners[i][1];
        num_landmarks++;
    }
    // Inner room corners
    float inner_corners[4][2] = {
        {inner_room.x_min, inner_room.y_min},
        {inner_room.x_max, inner_room.y_min},
        {inner_room.x_max, inner_room.y_max},
        {inner_room.x_min, inner_room.y_max}
    };
    for (int i = 0; i < 4; ++i) {
        landmarks[num_landmarks]    = malloc(sizeof(LandmarkMeasurement));
        landmarks[num_landmarks]->id = num_landmarks;
        landmarks[num_landmarks]->x = inner_corners[i][0];
        landmarks[num_landmarks]->y = inner_corners[i][1];
        num_landmarks++;
    }

    // --- Path: Four hallways around the inner room ---
    // Start at (2,2), go right, up, left, down (around the inner room)
    int num_poses = 0;
    GraphNode *head = malloc(sizeof(GraphNode));
    head->pose.x = 2.0f; head->pose.y = 2.0f; head->pose.theta = 0.0f;
    head->num_observations = 0;
    head->observations = NULL;
    head->next = NULL;
    GraphNode *current = head;
    num_poses++;

    // Define waypoints for the four hallways (with some noise)
    float waypoints[5][3] = {
        {16.0f, 2.0f, 0.0f},    // right hallway
        {16.0f, 16.0f, M_PI_2}, // up hallway
        {2.0f, 16.0f, M_PI},    // left hallway
        {2.0f, 2.0f, -M_PI_2},  // down hallway (back to start)
        {2.0f, 2.0f, 0.0f}      // loop closure
    };

    for (int w = 0; w < 5; ++w) {
        float dx = waypoints[w][0] - current->pose.x;
        float dy = waypoints[w][1] - current->pose.y;
        float dtheta = waypoints[w][2] - current->pose.theta;

        // Add noise to motion
        OdometryEdge control;
        control.dx = dx + randn(0.0f, 0.1f);
        control.dy = dy + randn(0.0f, 0.1f);
        control.dtheta = dtheta + randn(0.0f, 0.02f);

        // Predict motion and create new node
        GraphNode *new_node = predict_motion(current, control);
        num_poses++;
        current = new_node;

        // Add observations to visible landmarks (simulate range/bearing with noise)
        current->num_observations = 0;
        current->observations = NULL;
        for (int l = 0; l < num_landmarks; ++l) {
            float lx = landmarks[l]->x;
            float ly = landmarks[l]->y;
            // Only observe landmarks not blocked by the inner room
            if (!is_in_Obstacle((current->pose.x + lx)/2, (current->pose.y + ly)/2, &inner_room)) {
                float dx_lm = lx - current->pose.x;
                float dy_lm = ly - current->pose.y;
                float range = sqrtf(dx_lm*dx_lm + dy_lm*dy_lm) + randn(0.0f, 0.05f);
                float bearing = atan2f(dy_lm, dx_lm) - current->pose.theta + randn(0.0f, 0.01f);

                Observation obs;
                obs.landmark = landmarks[l];
                obs.landmark_index = l;
                obs.range = range;
                obs.bearing = bearing;
                add_observation(current, obs);
            }
        }
    }

    // --- Run SLAM ---
    gauss_newton_slam(head, landmarks, num_poses, num_landmarks, 10, 1e-4f);

    // --- Print results ---
    printf("Final poses:\n");
    GraphNode *node = head;
    int idx = 0;
    while (node != NULL) {
        printf("Pose %d: x=%.2f, y=%.2f, theta=%.2f\n", idx, node->pose.x, node->pose.y, node->pose.theta);
        node = node->next;
        idx++;
    }
    printf("Final landmarks:\n");
    for (int l = 0; l < num_landmarks; ++l) {
        printf("Landmark %d: x=%.2f, y=%.2f\n", l, landmarks[l]->x, landmarks[l]->y);
    }

    // --- Free memory ---
    node = head;
    while (node != NULL) {
        GraphNode *next = node->next;
        if (node->observations) free(node->observations);
        free(node);
        node = next;
    }
    for (int l = 0; l < num_landmarks; ++l) {
        free(landmarks[l]);
    }
}



// void example_ransac_2(void);
// void example_ransac(void);
// void example_ransac_2(void);

int main(void) {

	// rrt_star_demo();
    example_rectangle_room_with_inner_Obstacle();

    // example_coord_transform();
	// example_coord_transform_2();


	// example_ransac();
	// example_ransac_2();

  	return 0;
}
