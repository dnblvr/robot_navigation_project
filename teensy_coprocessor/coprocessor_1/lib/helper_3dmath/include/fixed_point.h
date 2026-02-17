

#ifdef  _FIXED_POINT_H_
#define _FIXED_POINT_H_

// Int            Fractional
// xxxxxxxxxxxxxx.xxxxxxxxxxxxxxxxxx

#define FIXED_PT_INT_TYPE       int32_t
#define FIXED_PT_INT_XL_TYPE    int64_t

typedef FIXED_PT_INT_TYPE       fp_int_t;
typedef FIXED_PT_INT_XL_TYPE    fp_intxl_t;

#define FIXED_PT_INT_BITS       14
#define FIXED_PT_NUM_BITS       (sizeof(fp_int_t) << 3)
#define FIXED_PT_FRAC_BITS      (FIXED_PT_NUM_BITS - FIXED_PT_INT_BITS)

#define FIXED_PT_SCALE          (FIXED_PT_FRAC_BITS)
#define FIXED_PT_SCALE_FACTOR   (1 << FIXED_PT_SCALE)

#define FIXED_PT_INT_MASK       (((1<<FIXED_PT_INT_BITS) - 1)  <<  FIXED_PT_FRAC_BITS)
#define FIXED_PT_FRAC_MASK      ((1<<FIXED_PT_FRAC_BITS) - 1)

// values
#define FIXED_PT_FROM_INT(a)    (a << FIXED_PT_SCALE)
#define FIXED_PT_VAL_0          0
#define FIXED_PT_VAL_HALF       (1 << (FIXED_PT_FRAC_BITS-1))
#define FIXED_PT_VAL_1          FIXED_PT_FROM_INT(1)
#define FIXED_PT_VAL_NEG_1      FIXED_PT_FROM_INT(-1)

#define FIXED_PT_SIGN_BIT(a)    ((a >> (FIXED_PT_NUM_BITS-1)) & 1)
#define FIXED_PT_SIGN(a)        (FIXED_PT_SIGN_BIT(a) == 0 ? FIXED_PT_VAL_1 : FIXED_PT_VAL_NEG_1)

float fp_to_float(fp_int_t a) {
  return (float)a / FIXED_PT_SCALE_FACTOR;
}

fp_int_t float_to_fp(float a) {
  return (fp_int_t)(a * FIXED_PT_SCALE_FACTOR);
}

fp_int_t fp_mul(fp_int_t a, fp_int_t b) {
  return ((fp_intxl_t)a * b) >> FIXED_PT_SCALE;
}

fp_int_t fp_div(fp_int_t a, fp_int_t b) {
  return ((fp_intxl_t)(a) << FIXED_PT_SCALE) / b;
}

fp_int_t fp_abs(fp_int_t a) {
  if (FIXED_PT_SIGN_BIT(a) == 0) {
    return a;
  }
  return ~a + 1;
}

fp_int_t fp_frac(fp_int_t a) {
  return fp_abs(a) & FIXED_PT_FRAC_MASK;
}

fp_int_t fp_floor(fp_int_t a) {
  fp_int_t frac = fp_frac(a);
  if (frac == 0) {
    return a;
  }

  if (FIXED_PT_SIGN_BIT(a) == 0) {
    return FIXED_PT_INT_MASK & a;
  } else {
    return (FIXED_PT_INT_MASK & a) + FIXED_PT_VAL_1;
  }
}

fp_int_t fp_ceil(fp_int_t a) {
  fp_int_t frac = fp_frac(a);
  if (frac == 0) {
    return a;
  }
  return fp_floor(a) + FIXED_PT_SIGN(a);
}

fp_int_t fp_round(fp_int_t a) {
  fp_int_t frac = fp_frac(a);

  if (frac >= FIXED_PT_VAL_HALF) {
    return fp_ceil(a);
  }

  return fp_floor(a);
}

#endif // _FIXED_POINT_H_
