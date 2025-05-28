#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

typedef struct {
    float x, y;
} Point2D;

// Helper: Find the index of the closest point in target for a given source point
int find_closest_point(Point2D src, Point2D* target, int target_size) {

    // counter variables
    int     i;

    float   min_dist    = FLT_MAX;
    int     min_idx     = 0;
    
    for (i = 0; i < target_size; i++) {
        float dx = src.x - target[i].x;
        float dy = src.y - target[i].y;
        float dist = dx*dx + dy*dy;
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    return min_idx;
}

// Helper: Compute centroid of a point set
void compute_centroid(Point2D* pts, int n, Point2D* centroid) {
    // counter variables
    int     i;

    centroid->x = 0.0f;
    centroid->y = 0.0f;
    for (i = 0; i < n; i++) {
        centroid->x += pts[i].x;
        centroid->y += pts[i].y;
    }
    centroid->x /= n;
    centroid->y /= n;
}

// Helper: SVD for 2x2 matrix (for rotation)
void svd_2x2(float a, float b, float c, float d, float* cs, float* sn) {
    // Compute SVD of [a b; c d] for 2D rotation
    float theta = atan2f(b - c, a + d);
    *cs = cosf(theta / 2.0f);
    *sn = sinf(theta / 2.0f);
}

// Main ICP function
void icp_2d(
    Point2D* source, int source_size,
    Point2D* target, int target_size,

    int     max_iterations,
    float   tolerance,
    float  *out_R,   // 2x2 rotation matrix (row-major)
    float  *out_t)   // 2x1 translation vector
{   
    
    // counter variables
    int     iter, i;


    // Initialize transformation
    float R[2][2]   = { {1, 0}, {0, 1} };
    float t[2]      = {0, 0};

    Point2D* src_trans = (Point2D*)malloc(source_size * sizeof(Point2D));
    for (i = 0; i < source_size; i++) {
        src_trans[i] = source[i];
    }

    float prev_error = FLT_MAX;
    for (iter = 0; iter < max_iterations; iter++) {
        // 1. Find correspondences
        int *correspondences = (int*)malloc(source_size * sizeof(int));
        for (i = 0; i < source_size; i++) {
            correspondences[i] = find_closest_point(src_trans[i], target, target_size);
        }

        // 2. Compute centroids
        Point2D centroid_src = {0,0}, centroid_tgt = {0,0};
        compute_centroid(src_trans, source_size, &centroid_src);
        Point2D* tgt_corr = (Point2D*)malloc(source_size * sizeof(Point2D));
        for (i = 0; i < source_size; i++) {
            tgt_corr[i] = target[correspondences[i]];
        }
        compute_centroid(tgt_corr, source_size, &centroid_tgt);

        // 3. Compute cross-covariance matrix
        float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
        for (i = 0; i < source_size; i++) {
            float xs = src_trans[i].x - centroid_src.x;
            float ys = src_trans[i].y - centroid_src.y;
            float xt =  tgt_corr[i].x - centroid_tgt.x;
            float yt =  tgt_corr[i].y - centroid_tgt.y;
            Sxx += xs * xt;
            Sxy += xs * yt;
            Syx += ys * xt;
            Syy += ys * yt;
        }

        // 4. Compute rotation (using SVD for 2x2)
        float det = Sxx * Syy - Sxy * Syx;
        float theta = atan2f(Sxy - Syx, Sxx + Syy);
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);

        float R_iter[2][2] = {
            {cos_theta, -sin_theta},
            {sin_theta,  cos_theta}
        };

        // 5. Compute translation
        float t_iter[2];
        t_iter[0] = centroid_tgt.x - (R_iter[0][0]*centroid_src.x + R_iter[0][1]*centroid_src.y);
        t_iter[1] = centroid_tgt.y - (R_iter[1][0]*centroid_src.x + R_iter[1][1]*centroid_src.y);

        // 6. Transform source points
        for (i = 0; i < source_size; i++) {
            float x = source[i].x;
            float y = source[i].y;
            src_trans[i].x = R_iter[0][0]*x + R_iter[0][1]*y + t_iter[0];
            src_trans[i].y = R_iter[1][0]*x + R_iter[1][1]*y + t_iter[1];
        }

        // 7. Check error
        float mean_error = 0.0f;
        for (i = 0; i < source_size; i++) {
            float dx = src_trans[i].x - tgt_corr[i].x;
            float dy = src_trans[i].y - tgt_corr[i].y;
            mean_error += sqrtf(dx*dx + dy*dy);
        }
        mean_error /= source_size;

        if (fabsf(prev_error - mean_error) < tolerance) {
            // Converged
            break;
        }
        prev_error = mean_error;

        // Update transformation (compose)
        float R_new[2][2];
        R_new[0][0] = R_iter[0][0]*R[0][0] + R_iter[0][1]*R[1][0];
        R_new[0][1] = R_iter[0][0]*R[0][1] + R_iter[0][1]*R[1][1];
        R_new[1][0] = R_iter[1][0]*R[0][0] + R_iter[1][1]*R[1][0];
        R_new[1][1] = R_iter[1][0]*R[0][1] + R_iter[1][1]*R[1][1];

        float t_new[2];
        t_new[0] = R_iter[0][0]*t[0] + R_iter[0][1]*t[1] + t_iter[0];
        t_new[1] = R_iter[1][0]*t[0] + R_iter[1][1]*t[1] + t_iter[1];

        R[0][0] = R_new[0][0]; R[0][1] = R_new[0][1];
        R[1][0] = R_new[1][0]; R[1][1] = R_new[1][1];
        t[0] = t_new[0]; t[1] = t_new[1];

        free(correspondences);
        free(tgt_corr);
    }

    // Output
    out_R[0] = R[0][0]; out_R[1] = R[0][1];
    out_R[2] = R[1][0]; out_R[3] = R[1][1];
    out_t[0] = t[0];
    out_t[1] = t[1];

    free(src_trans);
}