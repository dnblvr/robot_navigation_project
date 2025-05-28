// ekf.c

#ifndef __EKF_H__
#define __EKF_H__



void ekf_init(EKF* ekf, float wheelbase);

void ekf_predict(EKF* ekf, float delta_left, float delta_right);

void ekf_update(EKF* ekf, float gps_x, float gps_y);

void example_ekf();

#endif // __EKF_H__