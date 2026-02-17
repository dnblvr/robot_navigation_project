#include "fixed_point.c"

int main(void) {
    // Example usage of fixed-point functions
    float original = 3.14159f;
    fp_int_t fixed = float_to_fp(original);
    float converted = fp_to_float(fixed);

    printf("\n\n");

    printf("Original: %f, Fixed: %d, Converted: %f\n", 
            original, fixed, converted);

    printf("\n\n");

    return 0;
}