# run your script here Nabin

"""
Invariant Extended Kalman Filter (IEKF) on SE(2)

This script estimates the 2D pose (x, y, yaw) of a mobile robot
using an Invariant Extended Kalman Filter.

Key ideas:
- The robot pose is represented on the Lie group SE(2)
- Estimation errors live in the Lie algebra se(2)
- This improves consistency compared to a standard EKF

State:
    X ∈ SE(2)   -> 3x3 homogeneous transformation matrix
    P ∈ R³ˣ³   -> covariance of error in Lie algebra coordinates

Sensors (simulated):
    - Control input: linear velocity v, angular velocity w
    - GPS position measurements (x, y)
"""

import numpy as np

# ============================================================
# Lie Group Utilities for SE(2)
# ============================================================

def hat(xi):
    """
    Convert a vector in se(2) to its matrix (hat) representation.

    xi = [rho_x, rho_y, theta]
    """
    x, y, th = xi
    return np.array([
        [0,   -th,  x],
        [th,   0,   y],
        [0,    0,   0]
    ])

def vee(X):
    """
    Convert a matrix in se(2) to its vector (vee) representation.
    """
    return np.array([X[0, 2], X[1, 2], X[1, 0]])

def exp_se2(xi):
    """
    Exponential map from se(2) to SE(2).

    Converts a small pose increment in the Lie algebra
    into a valid SE(2) transformation.
    """
    x, y, th = xi
    T = np.eye(3)

    # Small-angle approximation
    if abs(th) < 1e-6:
        T[0:2, 2] = [x, y]
    else:
        A = np.sin(th) / th
        B = (1 - np.cos(th)) / th

        # Rotation matrix
        R = np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th),  np.cos(th)]
        ])

        # V matrix couples translation and rotation
        V = np.array([
            [A, -B],
            [B,  A]
        ])

        T[0:2, 0:2] = R
        T[0:2, 2] = V @ np.array([x, y])

    return T

def log_se2(T):
    """
    Logarithm map from SE(2) to se(2).

    Extracts the minimal error representation from a pose matrix.
    """
    th = np.arctan2(T[1, 0], T[0, 0])
    t = T[0:2, 2]

    if abs(th) < 1e-6:
        return np.array([t[0], t[1], 0.0])

    A = np.sin(th) / th
    B = (1 - np.cos(th)) / th

    V_inv = (1 / (A*A + B*B)) * np.array([
        [ A,  B],
        [-B,  A]
    ])

    rho = V_inv @ t
    return np.array([rho[0], rho[1], th])

# ============================================================
# IEKF Class Definition
# ============================================================

class IEKF_SE2:
    """
    Invariant Extended Kalman Filter for SE(2) pose estimation.
    """

    def __init__(self):
        # State estimate as a homogeneous transformation matrix
        self.X = np.eye(3)

        # Covariance of the invariant error
        self.P = np.eye(3) * 0.1

        # Process noise covariance (motion uncertainty)
        self.Q = np.diag([0.02, 0.02, 0.01])

        # Measurement noise covariance (GPS)
        self.R = np.diag([0.5, 0.5])

    def predict(self, v, w, dt):
        """
        Prediction step.

        Propagates the state using body-frame velocities:
            v  - linear velocity
            w  - angular velocity
            dt - time step
        """
        # Control input expressed in Lie algebra coordinates
        u = np.array([v * dt, 0.0, w * dt])

        # Left-invariant state propagation
        self.X = self.X @ exp_se2(u)

        # In IEKF, the Jacobian is state-independent
        F = np.eye(3)

        # Covariance propagation
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z):
        """
        Measurement update using GPS position.

        z = [x, y]
        """
        # Measurement Jacobian
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        # Predicted position from current state
        y_hat = self.X[0:2, 2]

        # Innovation (measurement residual)
        innovation = z - y_hat

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Correction expressed in Lie algebra
        delta_xi = K @ innovation

        # Left-invariant state update
        self.X = exp_se2(delta_xi) @ self.X

        # Covariance update
        self.P = (np.eye(3) - K @ H) @ self.P

    def pose(self):
        """
        Return pose in (x, y, theta) form.
        """
        x = self.X[0, 2]
        y = self.X[1, 2]
        th = np.arctan2(self.X[1, 0], self.X[0, 0])
        return x, y, th

# ============================================================
# Simple Simulation / Test Loop
# ============================================================

def main():
    """
    Simulates a robot moving with constant velocity
    and occasional GPS updates.
    """
    iekf = IEKF_SE2()

    dt = 0.1      # time step
    v = 1.0       # linear velocity
    w = 0.2       # angular velocity

    for k in range(100):
        # Prediction step
        iekf.predict(v, w, dt)

        # GPS update every 10 steps
        if k % 10 == 0:
            gps = np.array([
                iekf.X[0, 2] + np.random.randn() * 0.5,
                iekf.X[1, 2] + np.random.randn() * 0.5
            ])
            iekf.update_gps(gps)

        x, y, th = iekf.pose()
        print(f"t={k*dt:.1f}s | x={x:.2f}, y={y:.2f}, θ={th:.2f}")

if __name__ == "__main__":
    main()
