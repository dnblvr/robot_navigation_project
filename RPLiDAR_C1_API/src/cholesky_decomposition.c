/**
 * @file cholesky_decomposition.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * 
 */

#include "../inc/cholesky_decomposition.h"


int8_t cholesky_decompose(
    float*  A, 
    float*  L, 
    int     n)
{
    // counter variables
    int i, j, k;

    for (i = 0; i < n; ++i) {

        for (j = 0; j <= i; ++j) {

            float sum = A[i*n + j];

            for (k = 0; k < j; ++k)
                sum -= L[i*n + k] * L[j*n + k];


            // if at diagonal position
            if (i == j) {

                if (sum <= 0.0f)
                    return -1; // Not positive definite

                L[i*n + j]  = sqrtf(sum);

            // if not at any diagonal positions
            } else {

                L[i*n + j]  = sum / L[j*n + j];

            }
        }

        // Fill upper triangle with zeros for clarity
        for (j = i+1; j < n; ++j)
            L[i*n + j] = 0.0f;
    }

    return 0;
}


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

void forward_substitution(
        float*  L, 
        float*  b, 
        float*  y, 
        int     n)
{
    // counter variables
    int i, j;

    for (i = 0; i < n; ++i) {
        float sum = b[i];

        for (j = 0; j < i; ++j) {
            sum -= L[i*n + j] * y[j];
        }
        y[i] = sum / L[i*n + i];
    }
}


void backward_substitution(
        float  *L, 
        float  *y, 
        float  *x, 
        int     n)
{
    // counter variables
    int i, j;

    for (i = n-1; i >= 0; --i) {
        float sum = y[i];

        for (j = i+1; j < n; ++j) {
            sum -= L[j*n + i] * x[j];
        }

        x[i] = sum / L[i*n + i];
    }
}
