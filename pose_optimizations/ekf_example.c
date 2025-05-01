#include "pose_optimizations/inc/ekf.h"

int example_ekf() {

  EKF ekf;
  ekf_init(&ekf, 0.5f);  // 0.5m wheelbase

  while(1) {
    // Get encoder readings (example values)
    float dl = get_left_encoder();
    float dr = get_right_encoder();
    ekf_predict(&ekf, dl, dr);
    
    if (gps_available()) {
      float gx = get_gps_x();
      float gy = get_gps_y();
      ekf_update(&ekf, gx, gy);
    }
    
    // Use updated state
    printf("Pose: x=%.2f, y=%.2f, θ=%.2f\n", 
           ekf.x[0], ekf.x[1], ekf.x[2]);
  }
}