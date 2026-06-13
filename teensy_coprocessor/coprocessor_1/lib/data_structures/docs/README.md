
# Explanation of Madgwick Filter Implementation

## Scalar Math Function Equivalents

Use these functions when you need to replace a standard single-value calculation (scalar) with an optimized, hardware-accelerated, fast-scalar math version. [](https://arm-software.github.io/CMSIS-DSP/main/group__groupFastMath.html)

![alt text](https://encrypted-tbn1.gstatic.com/faviconV2?url=https://arm-software.github.io&client=AIM&size=128&type=FAVICON&fallback_opts=TYPE,SIZE,URL) GitHub Pages documentation

|      `<math.h>` Function       |   CMSIS-DSP Equivalent (f32)   |                       Notes / Description                        |
| ------------------------------ | ------------------------------ | ---------------------------------------------------------------- |
| `sin(x)` / `sinf(x)`           | `arm_sin_f32(x)`               | Linear interpolation via 512-entry table                         |
| `cos(x)` / `cosf(x)`           | `arm_cos_f32(x)`               | Linear interpolation via 512-entry table                         |
| `atan2(y, x)` / `atan2f(y, x)` | `arm_atan2_f32(y, x, &result)` | Computes arc tangent of $y / x$ using quadrant detection         |
| `sqrt(x)` / `sqrtf(x)`         | `arm_sqrt_f32(x, &result)`     | Uses the MCU's FPU hardware square-root instruction if available |

_Note: For fixed-point projects, CMSIS-DSP also provides `_q31` and `_q15` variations for these scalar functions (e.g., `arm_sin_q31()`)._

* * *

Array & Vector Processing Equivalents

If you are looping through an array to apply a `<math.h>` operation to every element, replace your loop entirely with these array-based vector operations to maximize hardware efficiency. [](https://arm-software.github.io/CMSIS-DSP/v1.14.1/)

![](https://encrypted-tbn1.gstatic.com/faviconV2?url=https://arm-software.github.io&client=AIM&size=128&type=FAVICON&fallback_opts=TYPE,SIZE,URL) GitHub Pages documentation +1

| Loop with `<math.h>` | CMSIS-DSP Vector Function (f32) | Notes / Description |
| --- | --- | --- |
| `fabs(x)` | `arm_abs_f32(pSrc, pDst, blockSize)` | Computes the absolute value of each element in a vector |
| `exp(x)` / `expf(x)` | `arm_vexp_f32(pSrc, pDst, blockSize)` | Computes the exponential $\exp(x)$ for an entire vector array |
| `log(x)` / `logf(x)` | `arm_vlog_f32(pSrc, pDst, blockSize)` | Computes the natural logarithm $\ln(x)$ for a vector array |
| `sqrt(x)` / `sqrtf(x)` | `arm_vsqrt_f32(pSrc, pDst, blockSize)` | Vectorized square root processing across an array block |

* * *

Basic Arithmetic Vector Replacements 

Standard loops utilizing operators like `+`, `-`, `*`, or `/` on arrays should be swapped with the following vector operations: [](https://arm-software.github.io/CMSIS-DSP/v1.11.0/modules.html)

![](https://encrypted-tbn1.gstatic.com/faviconV2?url=https://arm-software.github.io&client=AIM&size=128&type=FAVICON&fallback_opts=TYPE,SIZE,URL) GitHub Pages documentation

| Standard Loop Operation | CMSIS-DSP Vector Function | Description |
| --- | --- | --- |
| `y[i] = x1[i] + x2[i]` | `arm_add_f32(pSrcA, pSrcB, pDst, blockSize)` | Adds two vectors element-by-element |
| `y[i] = x1[i] - x2[i]` | `arm_sub_f32(pSrcA, pSrcB, pDst, blockSize)` | Subtracts vector B from vector A |
| `y[i] = x1[i] * x2[i]` | `arm_mult_f32(pSrcA, pSrcB, pDst, blockSize)` | Multiplies two vectors element-by-element |
| `y[i] = x1[i] / x2[i]` | `arm_div_num_f32(pSrcA, pSrcB, pDst, blockSize)` | Element-by-element array division |
| `y[i] = x[i] * constant` | `arm_scale_f32(pSrc, scale, pDst, blockSize)` | Multiplies an entire vector by a scalar constant |
| `y[i] = x[i] + constant` | `arm_offset_f32(pSrc, offset, pDst, blockSize)` | Adds a constant scalar value to a vector |

* * *

$$ \begin{aligned}
\big( \zeta^{\wedge} \big) ^ {\vee}
    = \begin{pmatrix}
        0 & -\omega & v_x \\
        \omega & 0 & v_y \\
        0 & 0 & 0
    \end{pmatrix} ^ {\vee}
    = \begin{pmatrix}
        v_x \\
        v_y \\
        \omega
    \end{pmatrix}
\end{aligned} $$
