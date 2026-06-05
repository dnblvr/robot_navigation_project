/**
 * @file cholesky_decomposition.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 */

#include <cholesky_decomposition.h>


int8_t Cholesky_Decompose(
        int     stride,
        int     n,
        float*  A, 
        
        float*  L)
{
    // counter variables
    int i, j, k;

    for (i = 0; i < n; ++i) {

        for (j = 0; j <= i; ++j) {

            float sum = A[i*stride + j];

            for (k = 0; k < j; ++k)
                sum -= L[i*stride + k] * L[j*stride + k];


            // if at diagonal position
            if (i == j) {

                if (sum <= 0.0f)
                    return -1; // Not positive definite

                L[i*stride + j]  = sqrtf(sum);

            // if not at any diagonal positions
            } else {

                L[i*stride + j]  = sum / L[j*stride + j];

            }
        }

        // Fill upper triangle with zeros for clarity
        for (j = i+1; j < n; ++j)
            L[i*stride + j] = 0.0f;
    }

    return 0;
}


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

void Cholesky_Forward_Substitution(
        int     stride,
        int     n,
        float*  L, 
        float*  b, 

        float*  y)
{
    // counter variables
    int i, j;

    for (i = 0; i < n; ++i) {

        float sum   = b[i];

        for (j = 0; j < i; ++j) {
            sum -= L[i*stride + j] * y[j];
        }

        y[i] = sum / L[i*stride + i];
    }

}


void Cholesky_Backward_Substitution(
        int     stride,
        int     n,
        float*  L,
        float*  y,

        float*  x)
{
    // counter variables
    int i, j;

    for (i = n-1; i >= 0; --i) {

        float sum = y[i];

        for (j = i+1; j < n; ++j) {
            sum -= L[j*stride + i] * x[j];
        }

        x[i] = sum / L[i*stride + i];
    }

}
