
#include "matrices.h"
#include "coordinate_transform.h"
#include "ransac.h"
#include "graphslam.h"
#include "rrt_star.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>



// void example_ransac_2(void);
void example_ransac(void);
void example_ransac_2(void);

int main(void) {

    // example_coord_transform();
	example_coord_transform_2();


	// example_ransac();
	// example_ransac_2();

  	return 0;
}






void example_ransac(void) {

	float sensor_points_matrix[3][15] = {
		{24.5,	12.4,	13,		13,		13.7,	14,		14.5,	16.1,	19,		21.6,	23.8,	25.2,	25.4,	26.4,	27.2},
		{18,	27.2,	25.7,	24.7,	22.4,	20.4,	17.6,	17.7,	18.5,	18.8,	19.7,	16.5,	14.5,	13,		10	},
		{1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	}
	};

	// Convert 2D array to 1D array for RANSAC processing
	float points[15][2] = {};  // 15 points, each with x and y coordinates

	for (uint8_t i = 0; i < 15; i++) {
		points[i][0] = sensor_points_matrix[0][i];  // x-coordinates
		points[i][1] = sensor_points_matrix[1][i];  // y-coordinates
	}

    uint8_t num_points = 15;
    ParametricLine lines[MAX_LINES];
    uint8_t num_lines = ransac_detect_lines(points, &num_points, lines);

    printf("Detected %d lines:\n", num_lines);
    for (uint8_t i = 0; i < num_lines; i++) {
        printf("Line %d: x(t) = %.2f + t * %.2f, y(t) = %.2f + t * %.2f\n",
               i + 1, lines[i].x0, lines[i].dx, lines[i].y0, lines[i].dy);
    }

    // Compute intersections between detected lines
    printf("\nIntersections:\n");
    for (uint8_t i = 0; i < num_lines; i++) {
        for (uint8_t j = i + 1; j < num_lines; j++) {
            float x, y;
            if (find_intersection(&lines[i], &lines[j], &x, &y)) {
                printf("Intersection between Line %d and Line %d: (%.2f, %.2f)\n", i + 1, j + 1, x, y);
            } else {
                printf("Line %d and Line %d are parallel or coincident.\n", i + 1, j + 1);
            }
        }
    }

    // Write points, lines, and intersections to a JSON file
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
        fprintf(json_file, "    {\"x0\": %.3f, \"dx\": %.3f, \"y0\": %.3f, \"dy\": %.3f}%s\n",
                lines[i].x0, lines[i].dx, lines[i].y0, lines[i].dy, (i < num_lines - 1) ? "," : "");
    }
    fprintf(json_file, "  ],\n");

	fprintf(json_file, "  \"intersections\": [\n");
	for (uint8_t i = 0; i < num_lines; i++) {
		for (uint8_t j = i + 1; j < num_lines; j++) {
			float x, y;
			if (find_intersection(&lines[i], &lines[j], &x, &y)) {
				fprintf(json_file, "    {\"x\": %.3f, \"y\": %.3f}%s\n", x, y,
						(i == num_lines - 2 && j == num_lines - 1) ? "" : ",");
			}
		}
	}
	fprintf(json_file, "  ]\n");
    fprintf(json_file, "}\n");

    fclose(json_file);

    printf("\nResults written to ransac_output.json\n");
}



void example_ransac_2(void) {
    float sensor_points_matrix[3][15] = {
        {24.5, 12.4, 13, 13, 13.7, 14, 14.5, 16.1, 19, 21.6, 23.8, 25.2, 25.4, 26.4, 27.2},
        {18, 27.2, 25.7, 24.7, 22.4, 20.4, 17.6, 17.7, 18.5, 18.8, 19.7, 16.5, 14.5, 13, 10},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    // Convert 2D array to 1D array for RANSAC processing
    float points[15][2] = {};  // 15 points, each with x and y coordinates

    for (uint8_t i = 0; i < 15; i++) {
        points[i][0] = sensor_points_matrix[0][i];  // x-coordinates
        points[i][1] = sensor_points_matrix[1][i];  // y-coordinates
    }

    uint8_t num_points = 15;
    ParametricLine lines[MAX_LINES];
    uint8_t num_lines = ransac_detect_lines(points, &num_points, lines);

    printf("Detected %d lines:\n", num_lines);
    for (uint8_t i = 0; i < num_lines; i++) {
        printf("Line %d: x(t) = %.2f + t * %.2f, y(t) = %.2f + t * %.2f\n",
               i + 1, lines[i].x0, lines[i].dx, lines[i].y0, lines[i].dy);
    }

    // Compute intersections between detected lines
    printf("\nIntersections:\n");
    for (uint8_t i = 0; i < num_lines; i++) {
        for (uint8_t j = i + 1; j < num_lines; j++) {
            float x, y;
            if (find_intersection(&lines[i], &lines[j], &x, &y)) {
                printf("Intersection between Line %d and Line %d: (%.2f, %.2f)\n", i + 1, j + 1, x, y);
            } else {
                printf("Line %d and Line %d are parallel or coincident.\n", i + 1, j + 1);
            }
        }
    }

    // Write points, lines, and intersections to a JSON file
    FILE *json_file = fopen("ransac_static_output.json", "w");
    if (!json_file) {
        perror("Failed to open JSON file");
        return;
    }

    fprintf(json_file, "{\n");
    fprintf(json_file, "  \"points\": [\n");
    for (uint8_t i = 0; i < 15; i++) {
        fprintf(json_file, "    {\"x\": %.3f, \"y\": %.3f}%s\n", points[i][0], points[i][1], (i < 14) ? "," : "");
    }
    fprintf(json_file, "  ],\n");

    fprintf(json_file, "  \"lines\": [\n");
    for (uint8_t i = 0; i < num_lines; i++) {
        fprintf(json_file, "    {\"x0\": %.3f, \"dx\": %.3f, \"y0\": %.3f, \"dy\": %.3f}%s\n",
                lines[i].x0, lines[i].dx, lines[i].y0, lines[i].dy, (i < num_lines - 1) ? "," : "");
    }
    fprintf(json_file, "  ],\n");

    fprintf(json_file, "  \"intersections\": [\n");
    uint8_t intersection_count = 0;
    for (uint8_t i = 0; i < num_lines; i++) {
        for (uint8_t j = i + 1; j < num_lines; j++) {
            float x, y;
            if (find_intersection(&lines[i], &lines[j], &x, &y)) {
                fprintf(json_file, "    {\"x\": %.3f, \"y\": %.3f}%s\n", x, y,
                        (intersection_count < (num_lines * (num_lines - 1)) / 2 - 1) ? "," : "");
                intersection_count++;
            }
        }
    }
    fprintf(json_file, "  ]\n");
    fprintf(json_file, "}\n");

    fclose(json_file);

    printf("\nResults written to ransac_static_output.json\n");
}