import numpy as np
import matplotlib.pyplot as plt
from ekf_se2 import EKF_SE2
from inekf_se2 import InEKF_SE2


# ============================================================
# Helper Utilities
# ============================================================

def wrap_angle(
        angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def chi2_threshold_1dof(
        alpha=0.95):
    """
    Return chi-squared threshold for 1 DOF.
    
    Parameters:
    -----------
    alpha : float
        Confidence level (default 0.95 for 95%)
    
    Returns:
    --------
    threshold : float
        Chi-squared critical value
    """
    # For 95% confidence, 1 DOF: 3.84
    # For 99% confidence, 1 DOF: 6.63
    if alpha == 0.95:
        return 3.84
    
    elif alpha == 0.99:
        return 6.63
    
    else:
        return 3.84  # Default


# ============================================================
# Bézier Curve Helpers
# ============================================================

# Control points for a 3-segment piecewise cubic Bézier loop.
# C1 continuity is enforced at every junction by the reflection rule:
#   first control point of segment N+1 = 2*junction - last control of segment N
#
# Segment 1: straight launch → gentle right-climbing arc
# Segment 2: tight S-bend at top  (high curvature → EKF stress region)
# Segment 3: long descending sweep back to origin (smooth loop closure)
#
# Initial tangent: B'(0) = 3*(P1-P0) = (3,0) → heading = 0  ✓ matches state[2]=0
# Loop closure:   B'(1,seg3) = 3*(P9-P8) = (3,0) = B'(0,seg1)  ✓ smooth
_BEZIER_SEGMENTS = [
    (np.array([0.0, 0.0]), np.array([1.0, 0.0]),   # seg 1
     np.array([5.0, 2.0]), np.array([5.0, 5.0])),
    (np.array([5.0, 5.0]), np.array([5.0, 8.0]),   # seg 2  (P4 = 2*P3 - P2)
     np.array([2.0, 9.0]), np.array([-1.0, 7.0])),
    (np.array([-1.0, 7.0]), np.array([-4.0, 5.0]), # seg 3  (P7 = 2*P6 - P5)
     np.array([-1.0, 0.0]), np.array([0.0, 0.0])),
]


def _bezier_cubic(P0, P1, P2, P3, t):
    """
    Cubic Bézier position + first two parametric derivatives at t ∈ [0, 1].

    Returns
    -------
    B   : (2,) position
    Bd  : (2,) first derivative  dB/dt
    Bdd : (2,) second derivative d²B/dt²
    """
    mt  = 1.0 - t
    B   =  mt**3*P0 + 3*mt**2*t*P1 + 3*mt*t**2*P2 + t**3*P3
    Bd  =  3*(mt**2*(P1 - P0) + 2*mt*t*(P2 - P1) + t**2*(P3 - P2))
    Bdd =  6*(mt*(P2 - 2*P1 + P0) +  t*(P3 - 2*P2 + P1))
    return B, Bd, Bdd


def _piecewise_bezier_commands(num_steps, dt,
                               segments=None,
                               target_speed=1.0):
    """
    Pre-compute (v_true, omega_true) for every simulation step from a
    piecewise cubic Bézier loop.

    Curvature κ = (Bd x Bdd) / |Bd|³ is exact (no linearisation).
    The whole path is then time-rescaled uniformly so mean linear speed
    equals target_speed — this preserves the omega/v ratio (curvature).

    The path is traversed once across the full num_steps window.

    Parameters
    ----------
    num_steps    : int
    dt           : float   simulation timestep (s)
    segments     : list of (P0,P1,P2,P3) ndarrays, or None → _BEZIER_SEGMENTS
    target_speed : float   desired mean linear speed (m/s)

    Returns
    -------
    v_arr, omega_arr : (num_steps,) float arrays
    """
    if segments is None:
        segments = _BEZIER_SEGMENTS

    n_seg     = len(segments)
    v_arr     = np.zeros(num_steps)
    omega_arr = np.zeros(num_steps)

    for k in range(num_steps):
        # t_global ∈ [0, n_seg) spans one full loop over num_steps steps
        t_global = k / num_steps * n_seg
        seg_idx  = min(int(t_global), n_seg - 1)
        t_local  = t_global - int(t_global)          # ∈ [0, 1)

        P0, P1, P2, P3 = segments[seg_idx]
        _, Bd, Bdd = _bezier_cubic(P0, P1, P2, P3, t_local)

        speed_param = np.linalg.norm(Bd)
        if speed_param < 1e-9:
            continue

        # Parametric → simulation-time scale factor
        scale         = n_seg / (num_steps * dt)
        v_arr[k]     = speed_param * scale
        # Signed curvature × speed = omega   (κ = cross / |Bd|³, ω = κ·v = cross/|Bd|²·scale)
        omega_arr[k] = (Bd[0]*Bdd[1] - Bd[1]*Bdd[0]) / (speed_param**2) * scale

    # Uniform time-rescale to hit target_speed
    mask   = v_arr > 1e-9
    mean_v = np.mean(v_arr[mask]) if mask.any() else 1.0
    ratio       = target_speed / mean_v
    v_arr      *= ratio
    omega_arr  *= ratio          # keeps ω/v = κ unchanged

    return v_arr, omega_arr


# ============================================================
# Ground Truth Simulation
# ============================================================

def simulate_ground_truth(
        dt,
        num_steps,
        path_type='circular',
        custom_velocities=None,
        wheelbase_L=0.3):
    """
    Generate ground truth trajectory with differential drive sensor measurements.
    
    Parameters:
    -----------
    dt : float
        Timestep in seconds
    num_steps : int
        Number of simulation steps
    path_type : str
        'circular', 'figure8', 'straight', 'bezier', or 'custom'
    custom_velocities : callable or None
        Function(t, state) -> (v, omega) for custom trajectories
    wheelbase_L : float
        Robot wheelbase in meters (default 0.3)
        
    Returns:
    --------
    gt_states : (N, 3) array of [x, y, theta]
    v_L_meas : (N,) left wheel velocity measurements (with noise)
    v_R_meas : (N,) right wheel velocity measurements (with noise)
    omega_gyro_meas : (N,) gyro angular velocity measurements (with noise)
    psi_measurements : (N,) magnetometer heading (with noise + bias)
    mag_norms : (N,) magnetometer magnitude (with soft iron variation)
    """
    
    # Initialize
    state           = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
    gt_states       = np.zeros((num_steps, 3))
    v_L_meas        = np.zeros(num_steps)
    v_R_meas        = np.zeros(num_steps)
    omega_gyro_meas = np.zeros(num_steps)
    psi_meas        = np.zeros(num_steps)
    mag_norms       = np.zeros(num_steps)
    
    # Pre-compute Bézier velocity commands (avoids recomputing inside the loop)
    if path_type == 'bezier':
        _bz_v, _bz_omega = _piecewise_bezier_commands(num_steps, dt)
    else:
        _bz_v = _bz_omega = None

    # Noise parameters
    wheel_noise       = 0.02   # Per-wheel velocity noise (m/s)
    gyro_noise        = 0.01   # Gyro noise (rad/s)
    mag_heading_noise = 0.1    # ~5.7 deg
    # mag_bias = 0.1      # **INTENTIONAL** Systematic heading bias (~5.7 deg)
                        # Simulates real magnetometer hard-iron distortion
                        # Filters SHOULD converge to this biased value!
    mag_bias = 0.0
    
    for k in range(num_steps):
        t = k * dt
        
        # Generate true velocities based on path type
        if path_type == 'circular':
            v_true      = 1.0 + 0.3 * np.sin(0.5 * t)
            omega_true  = 0.2
            
        elif path_type == 'figure8':
            v_true      = 1.0
            omega_true  = 0.4 * np.sin(0.3 * t)
            
        elif path_type == 'straight':
            v_true      = 1.0
            omega_true  = 0.05 * np.sin(0.1 * t)  # Small oscillations
            
        elif path_type == 'bezier':
            v_true      = _bz_v[k]
            omega_true  = _bz_omega[k]

        elif path_type == 'custom' and custom_velocities is not None:
            v_true, omega_true = custom_velocities(t, state)
            
        else:
            raise ValueError(f"Unknown path_type: {path_type}")
        
        # Integrate ground truth state
        theta = state[2]
        state[0] += v_true * np.cos(theta) * dt  # x
        state[1] += v_true * np.sin(theta) * dt  # y
        state[2] += omega_true * dt              # theta
        state[2] = wrap_angle(state[2])
        
        gt_states[k] = state.copy()
        
        # -------- Differential Drive Wheel Velocities --------
        # v_L = v - omega * L/2,  v_R = v + omega * L/2
        v_L_true    = v_true - omega_true * wheelbase_L / 2.0
        v_R_true    = v_true + omega_true * wheelbase_L / 2.0
        
        v_L_meas[k]        = v_L_true   + np.random.randn() * wheel_noise
        v_R_meas[k]        = v_R_true   + np.random.randn() * wheel_noise
        omega_gyro_meas[k] = omega_true + np.random.randn() * gyro_noise
        
        # Magnetometer heading (with bias and noise)
        psi_meas[k] = state[2] + mag_bias \
                     + np.random.randn()*mag_heading_noise
        psi_meas[k] = wrap_angle(psi_meas[k])
        
        # Magnetometer magnitude (soft iron: varies with heading)
        mag_norms[k] = 4500 + 500*np.sin(state[2]) + np.random.randn()*100
    
    return gt_states, v_L_meas, v_R_meas, omega_gyro_meas, psi_meas, mag_norms


# ============================================================
# Standard EKF Simulation
# ============================================================

def simulate_ekf(
        dt,
        num_steps,
        path_type='circular',
        update_steps: int = 5,
        wheelbase_L: float = 0.3,
        alpha: float = 0.5):
    """
    Run standard EKF simulation with magnetometer fusion.
    
    Parameters:
    -----------
    wheelbase_L : float
        Robot wheelbase in meters (default 0.3)
    alpha : float
        Complementary fusion weight for gyro omega (default 0.5)
    
    Returns:
    --------
    ekf_states : (N, 3) array
    uncertainties : (N,) array of trace(P)
    gate_history : (N,) array of bools (True if update applied)
    """
    # Generate ground truth and measurements
    gt_states, v_L_meas, v_R_meas, omega_gyro_meas, psi_meas, mag_norms = \
        simulate_ground_truth(dt, num_steps, path_type, wheelbase_L=wheelbase_L)
    
    # Initialize EKF
    process_noise = {'x': 0.05, 'y': 0.05, 'theta': 0.02}
    mag_noise = 0.1  # radians
    ekf = EKF_SE2(dt, process_noise, mag_noise, chi2_threshold=chi2_threshold_1dof())
    
    # Storage
    ekf_states = np.zeros((num_steps, 3))
    uncertainties = np.zeros(num_steps)
    gate_history = np.zeros(num_steps, dtype=bool)
    
    # Main loop
    for k in range(num_steps):
        # Prediction (always runs)
        ekf.predict(v_L_meas[k], v_R_meas[k], omega_gyro_meas[k], L=wheelbase_L, alpha=alpha)
        
        # Magnetometer update (conditional based on gating)
        # Update every 5 steps (20 Hz if prediction is 100 Hz)
        if k % update_steps == 0:
            gate_passed = ekf.update_magnetometer(psi_meas[k], mag_norms[k])
            gate_history[k] = gate_passed
        
        # Store results
        ekf_states[k] = ekf.get_state()
        uncertainties[k] = ekf.get_covariance_trace()
    
    return ekf_states, uncertainties, gate_history


# ============================================================
# Invariant EKF Simulation
# ============================================================

def simulate_inekf(
        dt, 
        num_steps, 
        path_type='circular',
        update_steps: int = 5,
        wheelbase_L: float = 0.3,
        alpha: float = 0.5):
    """
    Run Invariant EKF simulation with magnetometer fusion.
    
    Parameters:
    -----------
    wheelbase_L : float
        Robot wheelbase in meters (default 0.3)
    alpha : float
        Complementary fusion weight for gyro omega (default 0.5)

    Returns:
    --------
    inekf_states : (N, 3) array
    uncertainties : (N,) array of trace(P)
    gate_history : (N,) array of bools
    """
    # Generate ground truth and measurements
    gt_states, v_L_meas, v_R_meas, omega_gyro_meas, psi_meas, mag_norms = \
        simulate_ground_truth(dt, num_steps, path_type, wheelbase_L=wheelbase_L)
    
    # Initialize InEKF
    process_noise = {'x': 0.05, 'y': 0.05, 'theta': 0.02}
    mag_noise = 0.1  # radians
    inekf = InEKF_SE2(dt, process_noise, mag_noise, chi2_threshold=chi2_threshold_1dof())
    
    # Storage
    inekf_states    = np.zeros((num_steps, 3))
    uncertainties   = np.zeros(num_steps)
    gate_history    = np.zeros(num_steps, dtype=bool)
    
    # Main loop
    for k in range(num_steps):
        # Prediction (always runs)
        inekf.predict(v_L_meas[k], v_R_meas[k], omega_gyro_meas[k], L=wheelbase_L, alpha=alpha)
        
        # Magnetometer update (conditional)
        if k % update_steps == 0:
            gate_passed = inekf.update_magnetometer(psi_meas[k], mag_norms[k])
            gate_history[k] = gate_passed
        
        # Store results
        inekf_states[k] = inekf.get_state()
        uncertainties[k] = inekf.get_covariance_trace()
    
    return inekf_states, uncertainties, gate_history


# ============================================================
# Main Function
# ============================================================

if __name__ == "__main__":

    # Simulation parameters
    dt          = 0.01      # 100 Hz
    num_steps   = 5000      # 50 s  — covers ~2 full Bézier loops at 1 m/s
    path_type   = 'bezier'  # 'circular' | 'figure8' | 'straight' | 'bezier'
    update_rate = 3         # magnetometer update every 5 steps → 20 Hz

    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Differential drive and fusion parameters
    wheelbase_L = 0.3   # metres
    alpha       = 0.5   # 50/50 gyro / encoder omega fusion

    # Run simulations
    print("Running ground truth simulation...")
    gt_states, _, _, _, _, _ = simulate_ground_truth(
        dt, 
        num_steps, 
        path_type,
        wheelbase_L=wheelbase_L)
    
    print("Running Standard EKF simulation...")
    ekf_states, ekf_unc, ekf_gates = simulate_ekf(
            dt, 
            num_steps, 
            path_type, 
            update_steps=update_rate,
            wheelbase_L =wheelbase_L,
            alpha=alpha)
    
    print("Running Invariant EKF simulation...")
    inekf_states, inekf_unc, inekf_gates = simulate_inekf(
            dt, 
            num_steps, 
            path_type, 
            update_steps    = update_rate,
            wheelbase_L     = wheelbase_L,
            alpha           = alpha)
    

    # Compute statistics
    print(f"\n=== Results ===")
    print(f"EKF updates accepted: {np.sum(ekf_gates)}/{num_steps//update_rate}")
    print(f"InEKF updates accepted: {np.sum(inekf_gates)}/{num_steps//update_rate}")
    

    # Compute steady-state errors
    steady_start     = num_steps // 2  # After filter convergence
    ekf_mean_error   = np.mean(wrap_angle(  ekf_states[steady_start:, 2]
                                          -  gt_states[steady_start:, 2]))
    inekf_mean_error = np.mean(wrap_angle(  inekf_states[steady_start:, 2]
                                          -   gt_states[steady_start:, 2]))
    print(f"\nSteady-State Heading Errors (after convergence):")
    print(f"  EKF mean error: {np.rad2deg(ekf_mean_error):.2f}°")
    print(f"  InEKF mean error: {np.rad2deg(inekf_mean_error):.2f}°")
    

    # Compute RMSE over time for position (x, y)
    time = np.arange(num_steps) * dt
    ekf_position_error   = np.sqrt(  (ekf_states[:, 0] - gt_states[:, 0])**2
                                   + (ekf_states[:, 1] - gt_states[:, 1])**2)
    inekf_position_error = np.sqrt(  (inekf_states[:, 0] - gt_states[:, 0])**2
                                   + (inekf_states[:, 1] - gt_states[:, 1])**2)
    

    # Cumulative RMSE (rolling window)
    window_size = 75  # 0.5 second window at 100 Hz
    ekf_rmse    = np.zeros(num_steps)
    inekf_rmse  = np.zeros(num_steps)
    for k in range(num_steps):
        start_idx     = max(0, k - window_size + 1)
        ekf_rmse[k]   = np.sqrt(np.mean(  ekf_position_error[start_idx:k+1]**2))
        inekf_rmse[k] = np.sqrt(np.mean(inekf_position_error[start_idx:k+1]**2))
    
    # Plotting (2x2 grid)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes      = axes.flatten()  # Flatten to 1D for easier indexing
    

    # Plot 1: Trajectories
    axes[0].plot(gt_states[:, 0],
                 gt_states[:, 1],
                 'k-',
                 label='Ground Truth',
                 linewidth=2)
    axes[0].plot(ekf_states[:, 0],
                 ekf_states[:, 1],
                 'r--',
                 label='EKF',
                 alpha=0.7)
    axes[0].plot(inekf_states[:, 0],
                 inekf_states[:, 1],
                 'b--',
                 label='InEKF',
                 alpha=0.7)
    axes[0].set_xlabel('x [m]')
    axes[0].set_ylabel('y [m]')
    axes[0].set_title('2D Trajectories')
    axes[0].legend()
    axes[0].axis('equal')
    axes[0].grid()
    

    # Plot 2: Heading errors
    ekf_heading_error = wrap_angle(ekf_states[:, 2] - gt_states[:, 2])  # Signed error
    inekf_heading_error = wrap_angle(inekf_states[:, 2] - gt_states[:, 2])  # Signed error
    axes[1].plot(time, np.rad2deg(ekf_heading_error), 'r-', label='EKF', linewidth=1.5, alpha=0.8)
    axes[1].plot(time, np.rad2deg(inekf_heading_error), 'b-', label='InEKF', linewidth=1.5, alpha=0.8)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=1, label='Zero error', alpha=0.5)
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Heading Error [deg]')
    axes[1].set_title('Heading Estimation Error (Signed)')
    axes[1].legend()
    axes[1].grid()
    

    # Plot 3: Position RMSE over time
    axes[2].plot(time, ekf_rmse, 'r-', label='EKF', linewidth=1.5, alpha=0.8)
    axes[2].plot(time, inekf_rmse, 'b-', label='InEKF', linewidth=1.5, alpha=0.8)
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Position RMSE [m]')
    axes[2].set_title(f'Position RMSE (rolling window: {window_size} steps)')
    axes[2].legend()
    axes[2].grid()
    

    # Plot 4: Uncertainty (trace of P) & gate rejections
    axes[3].plot(time, ekf_unc, 'r-', label='EKF Uncertainty', alpha=0.7, linewidth=1.5)
    axes[3].plot(time, inekf_unc, 'b-', label='InEKF Uncertainty', alpha=0.7, linewidth=1.5)
    
    # Mark gate rejections as vertical lines
    # Only flag timesteps where an update was ATTEMPTED but rejected
    update_mask        = np.zeros(num_steps, dtype=bool)
    update_mask[::update_rate] = True
    reject_times_ekf   = time[update_mask & ~ekf_gates]
    reject_times_inekf = time[update_mask & ~inekf_gates]

    if len(reject_times_ekf) > 0:
        axes[3].vlines(reject_times_ekf, 0, max(ekf_unc), colors='r', alpha=0.2, 
                      linewidth=0.5, label='EKF Rejected')
    if len(reject_times_inekf) > 0:
        axes[3].vlines(reject_times_inekf, 0, max(inekf_unc), colors='b', alpha=0.2, 
                      linewidth=0.5, label='InEKF Rejected')
    
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Trace(P)')
    axes[3].set_title('Uncertainty & Gate Rejections')
    axes[3].legend()
    axes[3].grid()

    
    plt.tight_layout()
    plt.show()
    
    print("\nSimulation complete!")
