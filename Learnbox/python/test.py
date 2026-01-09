
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# EKF with IMU Acceleration for 2D Mobile Robot
# State: [x, y, ψ, v_x, v_y]
# ============================================================

class EKF_IMU_2D:
    def __init__(self, dt):
        self.dt = dt

        # State vector: position, heading, velocity (world frame)
        self.x = np.zeros((5, 1))

        # Covariance matrix
        self.P = np.eye(5) * 0.1

        # Process noise (IMU + unmodeled dynamics)
        self.Q = np.diag([
            0.01,   # x
            0.01,   # y
            0.005,  # ψ
            0.1,    # v_x
            0.1     # v_y
        ])

    def predict(self, a_body, yaw_rate):
        """
        EKF prediction using IMU acceleration.

        a_body   : [a_x, a_y] in robot body frame (m/s^2)
        yaw_rate : ψ̇ (rad/s)
        """

        dt = self.dt
        x, y, psi, vx, vy = self.x.flatten()
        ax_b, ay_b = a_body

        # Rotation: body → world
        c = np.cos(psi)
        s = np.sin(psi)

        ax_w = c * ax_b - s * ay_b
        ay_w = s * ax_b + c * ay_b

        # -------- Motion Model --------
        x_new   = x + vx * dt + 0.5 * ax_w * dt**2
        y_new   = y + vy * dt + 0.5 * ay_w * dt**2
        psi_new = psi + yaw_rate * dt

        vx_new  = vx + ax_w * dt
        vy_new  = vy + ay_w * dt

        self.x = np.array([[x_new], [y_new], [psi_new], [vx_new], [vy_new]])

        # -------- Jacobian --------
        F = np.eye(5)
        F[0, 3] = dt
        F[1, 4] = dt

        F[0, 2] = -0.5 * (ax_b * s + ay_b * c) * dt**2
        F[1, 2] =  0.5 * (ax_b * c - ay_b * s) * dt**2
        F[3, 2] = -(ax_b * s + ay_b * c) * dt
        F[4, 2] =  (ax_b * c - ay_b * s) * dt

        self.P = F @ self.P @ F.T + self.Q

    def update_position(self, z, R):
        """
        Position update (GPS / LiDAR localization)
        z : [x, y]
        R : measurement covariance
        """

        H = np.zeros((2, 5))
        H[0, 0] = 1
        H[1, 1] = 1

        z = np.array(z).reshape(2, 1)

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P


# ============================================================
# Simulation / Testing
# ============================================================

def simulate():
    dt = 0.1
    ekf = EKF_IMU_2D(dt)

    # Ground truth state
    x_gt = np.array([0.0, 0.0, 0.0, 1.0, 0.0])

    # Storage
    gt_path = []
    ekf_path = []

    # Noise
    imu_accel_noise = 0.05
    imu_gyro_noise = 0.01
    gps_noise = 0.3

    for k in range(300):
        t = k * dt

        # -------- Ground truth motion --------
        ax = 0.2 * np.cos(0.1 * t)
        ay = 0.0
        yaw_rate = 0.1

        psi = x_gt[2]
        c, s = np.cos(psi), np.sin(psi)

        ax_w = c * ax - s * ay
        ay_w = s * ax + c * ay

        x_gt[0] += x_gt[3] * dt + 0.5 * ax_w * dt**2
        x_gt[1] += x_gt[4] * dt + 0.5 * ay_w * dt**2
        x_gt[2] += yaw_rate * dt
        x_gt[3] += ax_w * dt
        x_gt[4] += ay_w * dt

        # -------- IMU measurements --------
        ax_m = ax + np.random.randn() * imu_accel_noise
        ay_m = ay + np.random.randn() * imu_accel_noise
        r_m  = yaw_rate + np.random.randn() * imu_gyro_noise

        ekf.predict([ax_m, ay_m], r_m)

        # -------- GPS / LiDAR update --------
        if k % 10 == 0:
            z = [
                x_gt[0] + np.random.randn() * gps_noise,
                x_gt[1] + np.random.randn() * gps_noise
            ]
            R = np.eye(2) * gps_noise**2
            ekf.update_position(z, R)

        gt_path.append(x_gt.copy())
        ekf_path.append(ekf.x.flatten())

    return np.array(gt_path), np.array(ekf_path)


# ============================================================
# Run Test
# ============================================================

if __name__ == "__main__":
    gt, ekf = simulate()

    plt.figure(figsize=(8, 6))
    plt.plot(gt[:, 0], gt[:, 1], 'k-', label="Ground Truth")
    plt.plot(ekf[:, 0], ekf[:, 1], 'r--', label="EKF Estimate")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("EKF with IMU Acceleration (2D)")
    plt.legend()
    plt.axis("equal")
    plt.grid()
    plt.show()
