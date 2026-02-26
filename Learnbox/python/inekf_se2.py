"""
Invariant EKF for SE(2) 2D Robot Localization
State: X in SE(2) (3x3 matrix Lie group)
Motion Model: Wheel odometry + gyro
Measurement Model: Magnetometer heading
"""

import numpy as np


# ============================================================
# SE(2) Lie Group Helper Functions (C-translatable)
# ============================================================

def wedge_se2(tau):
    """
    Hat operator: R^3 -> se(2) Lie algebra.
    
    tau = [vx, vy, omega] -> 3x3 skew-symmetric matrix
    
    Returns:
    --------
    tau_wedge : 3x3 ndarray
        [[0,     -omega, vx],
         [omega,  0,     vy],
         [0,      0,     0]]
    """
    vx, vy, omega = tau
    return np.array([
        [0.0,   -omega, vx],
        [omega,  0.0,   vy],
        [0.0,    0.0,   0.0]
    ])


def vee_se2(tau_wedge):
    """
    Vee operator: se(2) -> R^3.
    
    Inverse of wedge operator.
    
    Returns:
    --------
    tau : ndarray
        [vx, vy, omega]
    """
    vx      = tau_wedge[0, 2]
    vy      = tau_wedge[1, 2]
    omega   = tau_wedge[1, 0]
    return np.array([vx, vy, omega])


def exp_se2(tau):
    """
    Exponential map: R^3 -> SE(2).
    
    Maps tangent space vector to SE(2) matrix using closed-form solution.
    
    Parameters:
    -----------
    tau : array-like
        [vx, vy, omega] in tangent space
    
    Returns:
    --------
    X : 3x3 ndarray
        SE(2) matrix [[R, t], [0, 1]]
    """
    vx, vy, omega = tau
    
    # Handle small angle case
    if np.abs(omega) < 1e-6:
        # First-order Taylor approximation for omega -> 0
        # R ≈ I + omega * [omega]_x  (proper rotation)
        R = np.array([
            [1.0,   -omega, 0.0],
            [omega,  1.0,   0.0],
            [0.0,    0.0,   1.0]
        ])
        # V ≈ I for small omega: translation ≈ [vx, vy]
        t = np.array([[vx], [vy], [1.0]])

    else:
        # Closed-form SO(2) exponential
        c = np.cos(omega)
        s = np.sin(omega)
        
        # Rotation matrix
        R = np.array([
            [c,  -s,   0.0],
            [s,   c,   0.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Left Jacobian for SE(2)
        # V = (sin(omega)/omega) * I + ((1-cos(omega))/omega) * [omega]_x
        V = np.array([
            [    s/omega, -(1-c)/omega],
            [(1-c)/omega,      s/omega]
        ])
        
        # Translation: t = V * [vx, vy]
        v_vec = np.array([[vx], [vy]])
        t_xy = V @ v_vec
        t = np.array([[t_xy[0, 0]], [t_xy[1, 0]], [1.0]])
    
    # Construct SE(2) matrix
    X           = np.eye(3)
    X[0:2, 0:2] = R[0:2, 0:2]
    X[0:2, 2:3] = t[0:2, 0:1]
    
    return X


def log_se2(X):
    """
    Logarithm map: SE(2) -> R^3.
    
    Maps SE(2) matrix to tangent space vector.
    
    Parameters:
    -----------
    X : 3x3 ndarray
        SE(2) matrix
    
    Returns:
    --------
    tau : ndarray
        [vx, vy, omega]
    """
    # Extract rotation angle
    omega = np.arctan2(X[1, 0], X[0, 0])
    
    # Extract translation
    tx = X[0, 2]
    ty = X[1, 2]
    
    # Handle small angle case
    if np.abs(omega) < 1e-6:
        vx = tx
        vy = ty

    else:
        # Inverse of left Jacobian
        c = np.cos(omega)
        s = np.sin(omega)
        
        V_inv = (omega / (2 * (1 - c))) * np.array([
            [ s,       1 - c],
            [-(1 - c), s]
        ])
        
        t_vec   = np.array([[tx], [ty]])
        v_vec   = V_inv @ t_vec
        vx      = v_vec[0, 0]
        vy      = v_vec[1, 0]
    
    return np.array([vx, vy, omega])


def adjoint_se2(X):
    """
    Compute adjoint matrix for SE(2).
    
    Ad_X transforms tangent vectors between reference frames.
    
    Parameters:
    -----------
    X : 3x3 ndarray
        SE(2) matrix
    
    Returns:
    --------
    Ad : 3x3 ndarray
        Adjoint matrix
    """
    # Extract R and t
    R = X[0:2, 0:2]
    t = X[0:2, 2]
    
    # Construct adjoint in block form
    # Ad = [[R, [t]_x * R], [0, 1]]
    # For SE(2): [t]_x = [[0, -t[1]], [t[1], 0]] but we work in 3D tangent space
    
    Ad = np.zeros((3, 3))
    Ad[0:2, 0:2] =  R
    Ad[0, 2]     = -t[1]
    Ad[1, 2]     =  t[0]
    Ad[2, 2]     =  1.0
    
    return Ad


# ============================================================
# Invariant EKF Class
# ============================================================

class InEKF_SE2:
    def __init__(self,
                 dt,
                 process_noise,
                 mag_noise,
                 chi2_threshold=3.84):
        """
        Initialize Invariant EKF for SE(2).
        
        Parameters:
        -----------
        dt : float
            Timestep in seconds
        process_noise : dict
            {'x': float, 'y': float, 'theta': float} - process noise std devs
        mag_noise : float
            Magnetometer heading measurement noise (radians std dev)
        chi2_threshold : float
            Chi-squared threshold for innovation gating
        """
        self.dt = dt
        self.chi2_threshold = chi2_threshold
        
        # State: SE(2) matrix (3x3)
        self.X = np.eye(3)
        
        # Covariance in tangent space (3x3)
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance in tangent space
        self.Q = np.diag([
            process_noise['x']**2,
            process_noise['y']**2,
            process_noise['theta']**2
        ])
        
        # Measurement noise covariance
        self.R = np.array([[mag_noise**2]])
        
        # Magnitude gate limits
        self.mag_norm_min = 2000.0
        self.mag_norm_max = 6500.0

    
    def predict(self, v_L, v_R, omega_gyro, L=0.3, alpha=0.5):
        """
        InEKF prediction step on SE(2) manifold using differential drive
        kinematics with complementary gyro + encoder omega fusion.
        
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
        
        # -------- Differential Drive Kinematics --------
        v               = (v_L + v_R) / 2.0
        omega_encoders  = (v_R - v_L) / L
        omega           = alpha * omega_gyro + (1.0 - alpha) * omega_encoders
        
        # Construct input in tangent space
        # u = [v*dt, 0, omega*dt] (body-frame velocity integrated)
        u = np.array([v * dt, 0.0, omega * dt])
        
        # -------- State Propagation on Manifold --------
        # X_new = X @ exp(u)
        X_delta = exp_se2(u)
        self.X = self.X @ X_delta
        
        # -------- Covariance Propagation --------
        # P_new = Ad * P * Ad^T + Q
        # where Ad = adjoint of exp(u)
        Ad      = adjoint_se2(X_delta)
        self.P  = Ad @ self.P @ Ad.T + self.Q
    
    def update_magnetometer(self, psi_mag, mag_norm):
        """
        Magnetometer heading measurement update with gating.
        
        Parameters:
        -----------
        psi_mag : float
            Measured heading from magnetometer (radians)
        mag_norm : float
            Magnetometer magnitude
        
        Returns:
        --------
        gate_passed : bool
            True if update was applied
        """
        # -------- Pre-gate: Magnitude Check --------
        if mag_norm < self.mag_norm_min or mag_norm > self.mag_norm_max:
            return False
        
        # -------- Extract Current Heading from SE(2) State --------
        theta_hat = np.arctan2(self.X[1, 0], self.X[0, 0])
        
        # -------- Measurement Model --------
        # h(X) = theta (heading component)
        # Measurement Jacobian in tangent space: H = [0, 0, 1]
        H = np.array([[0.0, 0.0, 1.0]])
        
        # -------- Innovation --------
        y = self._wrap_angle(psi_mag - theta_hat)
        y = np.array([[y]])
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # -------- Innovation Gate --------
        mahalanobis = y.T @ np.linalg.inv(S) @ y
        if mahalanobis[0, 0] > self.chi2_threshold:
            return False
        
        # -------- Kalman Gain --------
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # -------- State Update on Manifold --------
        # Correction in tangent space
        delta_xi = (K @ y).flatten()  # [delta_vx, delta_vy, delta_omega]
        
        # Apply correction: X_new = X @ exp(delta_xi)
        X_correction = exp_se2(delta_xi)
        self.X = self.X @ X_correction
        
        # -------- Covariance Update (Joseph Form) --------
        I_KH = np.eye(3) - K @ H
        self.P =   I_KH @ self.P @ I_KH.T \
                 +    K @ self.R @ K.T
        
        return True
    
    def get_state(self):
        """
        Extract [x, y, theta] from SE(2) matrix.
        
        Returns:
        --------
        state : ndarray
            [x, y, theta] as (3,) array
        """
        x = self.X[0, 2]
        y = self.X[1, 2]
        theta = np.arctan2(self.X[1, 0], self.X[0, 0])
        return np.array([x, y, theta])
    
    def get_covariance_trace(self):
        """
        Get trace of covariance matrix.
        
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
