
// #include "../inc/ransac.h"
#include "ransac.h"

// Function to calculate the distance of a point from a line
float point_line_distance(float x, float y, LineModel* line) {
    return    fabsf(y - (line->m * x + line->c)) 
            / sqrtf(line->m * line->m + 1);
}



uint8_t ransac_detect_lines(float points[][2], uint8_t *num_points, LineModel lines[MAX_LINES]) {
    uint8_t num_lines = 0;

    while (*num_points > 0 && num_lines < MAX_LINES) {
        LineModel best_model = {0};
        uint8_t max_inliers = 0;

        // Dynamically allocate memory for inliers
        uint8_t *inliers = (uint8_t *)malloc(*num_points * sizeof(uint8_t));
        if (!inliers) {
            perror("Failed to allocate memory for inliers");
            return num_lines;
        }

#ifdef DEBUG_MODE
        FILE *debug_file = fopen("ransac_debug.json", num_lines == 0 ? "w" : "a");
        if (!debug_file) {
            perror("Failed to open debug file");
            free(inliers);
            return 0;
        }
        if (num_lines == 0) fprintf(debug_file, "[\n");  // Start JSON array
#endif

        // run RANSAC to find the best line
        for (int iter = 0; iter < RANSAC_MAX_ITER; iter++) {
            // randomly select two points
            uint8_t idx1 = rand() % *num_points;
            uint8_t idx2 = rand() % *num_points;

            if (idx1 == idx2) continue;  // Ensure two distinct points

            // compute line model (y = mx + c)
            float x1 = points[idx1][0], y1 = points[idx1][1];
            float x2 = points[idx2][0], y2 = points[idx2][1];

            if (fabsf(x2 - x1) < 1e-6) continue;  // Avoid division by zero (vertical line)

            LineModel model;
            model.m = (y2 - y1) / (x2 - x1);
            model.c = y1 - model.m * x1;

#ifdef DEBUG_MODE
            // Log current LineModel to JSON
            fprintf(debug_file, "  {\"iteration\": %d, \"m\": %.6f, \"c\": %.6f}", iter, model.m, model.c);
            if (iter < RANSAC_MAX_ITER - 1) fprintf(debug_file, ",\n");
#endif

            // Count inliers
            uint8_t current_inliers = 0;
            for (uint8_t i = 0; i < *num_points; i++) {
                float x = points[i][0], y = points[i][1];
                if (point_line_distance(x, y, &model) < INLIER_THRESHOLD) {
                    inliers[current_inliers++] = i;
                }
            }

            // Update best model if more inliers are found
            if (current_inliers > max_inliers) {
                max_inliers = current_inliers;
                best_model = model;
            }
        }

#ifdef DEBUG_MODE
        fprintf(debug_file, "\n]\n");  // End JSON array
        fclose(debug_file);
#endif

        // If no inliers were found, stop
        if (max_inliers == 0) {
            free(inliers);
            break;
        }

        // Save the detected line
        lines[num_lines++] = best_model;

        // Remove inliers from the dataset
        float new_points[*num_points - max_inliers][2];
        uint8_t new_num_points = 0;
        for (uint8_t i = 0; i < *num_points; i++) {
            uint8_t is_inlier = 0;
            for (uint8_t j = 0; j < max_inliers; j++) {
                if (i == inliers[j]) {
                    is_inlier = 1;
                    break;
                }
            }
            if (!is_inlier) {
                new_points[new_num_points][0] = points[i][0];
                new_points[new_num_points][1] = points[i][1];
                new_num_points++;
            }
        }

        // Update the dataset
        for (uint8_t i = 0; i < new_num_points; i++) {
            points[i][0] = new_points[i][0];
            points[i][1] = new_points[i][1];
        }
        *num_points = new_num_points;

        // Free dynamically allocated memory
        free(inliers);
    }

    return num_lines;
}



void example_ransac(void) {
    // Points forming lines based on the given equations
    float points[48][2] = {
        // Line 1: y = (sqrt(3)/3)x + 5
        {1.0, 5.577}, {2.0, 5.962}, {3.0, 6.346}, {4.0, 6.731}, {5.0, 7.115}, {6.0, 7.500}, {7.0, 7.885}, {8.0, 8.269},

        // Line 2: y = -(sqrt(3)/3)x + 20
        {1.0, 19.423}, {2.0, 19.038}, {3.0, 18.654}, {4.0, 18.269}, {5.0, 17.885}, {6.0, 17.500}, {7.0, 17.115}, {8.0, 16.731},

        // Line 3: y = -100x + 2500
        {20.0, 500.0}, {21.0, 400.0}, {22.0, 300.0}, {23.0, 200.0}, {24.0, 100.0}, {25.0, 0.0}, {26.0, -100.0}, {27.0, -200.0},

        // Line 4: y = (sqrt(3)/3)x - 20
        {25.0, -15.577}, {24.0, -15.962}, {23.0, -16.346}, {22.0, -16.731}, {21.0, -17.115}, {20.0, -17.500}, {19.0, -17.885}, {18.0, -18.269},

        // Line 5: y = 100x - 100
        {1.0, 0.0}, {2.0, 100.0}, {3.0, 200.0}, {4.0, 300.0}, {5.0, 400.0}, {6.0, 500.0}, {7.0, 600.0}, {8.0, 700.0},

        // Line 6: y = -(sqrt(3)/3)x - 5
        {1.0, -5.577}, {2.0, -5.962}, {3.0, -6.346}, {4.0, -6.731}, {5.0, -7.115}, {6.0, -7.500}, {7.0, -7.885}, {8.0, -8.269}
    };

    uint8_t num_points = 48;
    LineModel lines[MAX_LINES];
    uint8_t num_lines = ransac_detect_lines(points, &num_points, lines);

    printf("Detected %d lines:\n", num_lines);
    for (uint8_t i = 0; i < num_lines; i++) {
        printf("Line %d: y = %.2fx + %.2f\n", i + 1, lines[i].m, lines[i].c);
    }

    // Write points and lines to a JSON file
    FILE *json_file = fopen("ransac_output.json", "w");
    if (!json_file) {
        perror("Failed to open JSON file");
        return;
    }

    fprintf(json_file, "{\n");
    fprintf(json_file, "  \"points\": [\n");
    for (uint8_t i = 0; i < 48; i++) {
        fprintf(json_file, "    {\"x\": %.3f, \"y\": %.3f}%s\n", points[i][0], points[i][1], (i < 47) ? "," : "");
    }
    fprintf(json_file, "  ],\n");

    fprintf(json_file, "  \"lines\": [\n");
    for (uint8_t i = 0; i < num_lines; i++) {
        fprintf(json_file, "    {\"m\": %.3f, \"c\": %.3f}%s\n", lines[i].m, lines[i].c, (i < num_lines - 1) ? "," : "");
    }
    fprintf(json_file, "  ]\n");
    fprintf(json_file, "}\n");

    fclose(json_file);

    printf("Detected %d lines. Results written to ransac_output.json\n", num_lines);
}