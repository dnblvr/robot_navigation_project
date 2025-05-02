// ekf.h


#ifndef _EKF_H_
#define _EKF_H_

#include "matrices.h"

// Helper to create matrix views
#define MAT_VIEW(name, rows, cols, data) \
    Matrix name = {.rows=rows, .cols=cols, .data=data}

typedef struct {
  // State
  float x[3];  // [x, y, theta]
  
  // Covariance
  Matrix P;
  float P_data[9];  // 3x3
  
  // Noise
  Matrix Q;
  float Q_data[9];  // 3x3
  Matrix R;
  float R_data[4];  // 2x2
  
  // Models
  Matrix F;
  float F_data[9];  // 3x3
  Matrix H;
  float H_data[6];  // 2x3
  
  // Physical
  float wheelbase;
} EKF;

void ekf_init(EKF* ekf, float wheelbase);
void ekf_predict(EKF* ekf, float delta_left, float delta_right);
void ekf_update(EKF* ekf, float gps_x, float gps_y);

#endif  // _EKF_H_