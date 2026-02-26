"""
Standard EKF for SE(2) 2D Robot Localization
State: [x, y, theta] (3x1 vector)
Motion Model: Wheel odometry + gyro
Measurement Model: Magnetometer heading
"""

import numpy as np


class EKF_SE2:
    def __init__(self, dt, process_noise, mag_noise, chi2_threshold=3.84):
        """
        Initialize Standard EKF for SE(2).
        
        Parameters:
        -----------
        dt : float
            Timestep in seconds
        process_noise : dict
            {'x': float, 'y': float, 'theta': float} - process noise std devs
        mag_noise : float
            Magnetometer heading measurement noise (radians std dev)
        chi2_threshold : float
            Chi-squared threshold for innovation gating (default 3.84 for 95%, 1-DOF)
        """
        self.dt = dt
        self.chi2_threshold = chi2_threshold
        
        # State vector: [x, y, theta]
        self.x = np.zeros((3, 1))
        
        # Covariance matrix
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([
            process_noise['x']**2,
            process_noise['y']**2,
            process_noise['theta']**2
        ])
        
        # Measurement noise covariance (magnetometer heading)
        self.R = np.array([[mag_noise**2]])
        
        # Magnitude gate limits (from ICM20948 AK09916 magnetometer)
        self.mag_norm_min = 2000.0
        self.mag_norm_max = 6500.0
    
    def predict(self, v_L, v_R, omega_gyro, L=0.3, alpha=0.5):
        """
        EKF prediction step using differential drive kinematics
        with complementary gyro + encoder omega fusion.
        
        Parameters:
        -----------
        v_L : float
            Left wheel velocity (m/s) from encoder
        v_R : float
            Right wheel velocity (m/s) from encoder
        omega_gyro : float
            Angular velocity (rad/s) from gyroscope
        L : float
            Wheelbase (m), default 0.3
        alpha : float
            Complementary fusion weight for gyro (0=encoders only, 1=gyro only)
        """
        dt = self.dt
        x, y, theta = self.x.flatten()
        
        # -------- Differential Drive Kinematics --------
        v               = (v_L + v_R) / 2.0
        omega_encoders  = (v_R - v_L) / L
        omega           = alpha * omega_gyro + (1.0 - alpha) * omega_encoders
        
        # -------- Motion Model --------
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + omega * dt
        
        # Wrap angle to [-pi, pi]
        theta_new = self._wrap_angle(theta_new)
        
        self.x = np.array([[x_new], [y_new], [theta_new]])
        
        # -------- Jacobian (State Transition Matrix F) --------
        F = np.eye(3)
        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] =  v * np.cos(theta) * dt
        
        # -------- Covariance Prediction --------
        self.P = F @ self.P @ F.T + self.Q
    
    def update_magnetometer(self, psi_mag, mag_norm):
        """
        Magnetometer heading measurement update with gating.
        
        Parameters:
        -----------
        psi_mag : float
            Measured heading from magnetometer (radians)
        mag_norm : float
            Magnetometer magnitude (for pre-gating)
        
        Returns:
        --------
        gate_passed : bool
            True if update was applied, False if gated out
        """
        # -------- Pre-gate: Magnitude Check --------
        if mag_norm < self.mag_norm_min or mag_norm > self.mag_norm_max:
            return False
        
        # -------- Measurement Model --------
        # h(x) = theta (extract heading from state)
        theta = self.x[2, 0]
        
        # Measurement Jacobian
        H = np.array([[0.0, 0.0, 1.0]])
        
        # -------- Innovation --------
        y = self._wrap_angle(psi_mag - theta)
        y = np.array([[y]])  # Make it 1x1 matrix
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # -------- Innovation Gate --------
        # Mahalanobis distance: y^T * S^-1 * y
        mahalanobis = y.T @ np.linalg.inv(S) @ y
        if mahalanobis[0, 0] > self.chi2_threshold:
            return False
        
        # -------- Kalman Gain --------
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # -------- State Update --------
        self.x = self.x + K @ y
        self.x[2, 0] = self._wrap_angle(self.x[2, 0])
        
        # -------- Covariance Update (Joseph Form) --------
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        return True
    
    def get_state(self):
        """
        Get current state estimate.
        
        Returns:
        --------
        state : ndarray
            [x, y, theta] as (3,) array
        """
        return self.x.flatten()
    
    def get_covariance_trace(self):
        """
        Get trace of covariance matrix (total uncertainty).
        
        Returns:
        --------
        trace : float
            trace(P)
        """
        return np.trace(self.P)
    
    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
