import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from scipy.integrate import solve_ivp
import casadi as ca

class QuadrotorMPC:
    def __init__(self, dt=0.1, N=10):
        """
        Initialize MPC controller for quadrotor
        
        Args:
            dt: Time step
            N: Prediction horizon
        """
        self.dt = dt
        self.N = N
        
        # Quadrotor parameters
        self.params = {
            'mass': 0.5,      # kg
            'gravity': 9.81,   # m/s²
            'inertia': np.diag([0.01, 0.01, 0.02]),  # kg⋅m²
            'arm_length': 0.25,  # m
        }
        
        # State and input dimensions
        self.nx = 12  # State dimension
        self.nu = 4   # Input dimension
        
        # MPC weights
        self.Q = np.diag([10, 10, 10, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1])
        self.R = np.diag([0.1, 0.1, 0.1, 0.1])
        self.P = self.Q  # Terminal cost
        
        # Constraints
        self.u_min = np.array([0, -1, -1, -1])     # [thrust_min, τ_min]
        self.u_max = np.array([15, 1, 1, 1])       # [thrust_max, τ_max]
        
        # Initialize CasADi optimization
        self._setup_optimization()
    
    def _setup_optimization(self):
        """Setup CasADi optimization problem"""
        # Decision variables
        self.X = ca.MX.sym('X', self.nx, self.N + 1)  # States
        self.U = ca.MX.sym('U', self.nu, self.N)      # Controls
        self.X_ref = ca.MX.sym('X_ref', self.nx, self.N + 1)  # Reference
        
        # Cost function
        cost = 0
        
        # Stage costs
        for k in range(self.N):
            state_error = self.X[:, k] - self.X_ref[:, k]
            cost += ca.mtimes([state_error.T, self.Q, state_error])
            cost += ca.mtimes([self.U[:, k].T, self.R, self.U[:, k]])
        
        # Terminal cost
        terminal_error = self.X[:, self.N] - self.X_ref[:, self.N]
        cost += ca.mtimes([terminal_error.T, self.P, terminal_error])
        
        # Constraints
        g = []
        
        # Dynamics constraints
        for k in range(self.N):
            x_next = self._rk4_step(self.X[:, k], self.U[:, k])
            g.append(self.X[:, k+1] - x_next)
        
        # Create optimization problem
        opt_variables = ca.vertcat(
            ca.reshape(self.X, -1, 1),
            ca.reshape(self.U, -1, 1)
        )
        
        nlp = {
            'x': opt_variables,
            'f': cost,
            'g': ca.vertcat(*g),
            'p': ca.reshape(self.X_ref, -1, 1)
        }
        
        # Solver options
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.sb': 'yes'
        }
        
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
        
        # Bounds
        self._setup_bounds()
    
    def _setup_bounds(self):
        """Setup optimization bounds"""
        # State bounds (relaxed for simplicity)
        x_min = [-np.inf] * self.nx
        x_max = [np.inf] * self.nx
        
        # Input bounds
        u_min = self.u_min.tolist()
        u_max = self.u_max.tolist()
        
        # Create bounds for all variables
        self.lbx = []
        self.ubx = []
        
        # State bounds
        for k in range(self.N + 1):
            self.lbx.extend(x_min)
            self.ubx.extend(x_max)
        
        # Input bounds
        for k in range(self.N):
            self.lbx.extend(u_min)
            self.ubx.extend(u_max)
        
        # Constraint bounds (dynamics must be satisfied exactly)
        self.lbg = [0] * (self.nx * self.N)
        self.ubg = [0] * (self.nx * self.N)
    
    def _rk4_step(self, x, u):
        """Runge-Kutta 4th order integration step"""
        k1 = self._dynamics(x, u)
        k2 = self._dynamics(x + self.dt/2 * k1, u)
        k3 = self._dynamics(x + self.dt/2 * k2, u)
        k4 = self._dynamics(x + self.dt * k3, u)
        
        return x + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    
    def _dynamics(self, x, u):
        """Quadrotor dynamics model"""
        # Extract states
        pos = x[0:3]      # [x, y, z]
        euler = x[3:6]    # [φ, θ, ψ]
        vel = x[6:9]      # [ẋ, ẏ, ż]
        omega = x[9:12]   # [p, q, r]
        
        # Extract inputs
        T = u[0]          # Total thrust
        tau = u[1:4]      # Torques [τ_φ, τ_θ, τ_ψ]
        
        # Parameters
        m = self.params['mass']
        g = self.params['gravity']
        I = self.params['inertia']
        
        # Rotation matrix from body to world frame
        phi, theta, psi = euler[0], euler[1], euler[2]
        
        R_x = ca.MX([[1, 0, 0],
                     [0, ca.cos(phi), -ca.sin(phi)],
                     [0, ca.sin(phi), ca.cos(phi)]])
        
        R_y = ca.MX([[ca.cos(theta), 0, ca.sin(theta)],
                     [0, 1, 0],
                     [-ca.sin(theta), 0, ca.cos(theta)]])
        
        R_z = ca.MX([[ca.cos(psi), -ca.sin(psi), 0],
                     [ca.sin(psi), ca.cos(psi), 0],
                     [0, 0, 1]])
        
        R = ca.mtimes([R_z, R_y, R_x])
        
        # Thrust vector in world frame
        thrust_body = ca.MX([0, 0, T])
        thrust_world = ca.mtimes(R, thrust_body)
        
        # Position derivatives
        pos_dot = vel
        
        # Euler angle derivatives
        W = ca.MX([[1, ca.sin(phi)*ca.tan(theta), ca.cos(phi)*ca.tan(theta)],
                   [0, ca.cos(phi), -ca.sin(phi)],
                   [0, ca.sin(phi)/ca.cos(theta), ca.cos(phi)/ca.cos(theta)]])
        
        euler_dot = ca.mtimes(W, omega)
        
        # Velocity derivatives
        gravity_world = ca.MX([0, 0, -m * g])
        vel_dot = (thrust_world + gravity_world) / m
        
        # Angular velocity derivatives
        omega_dot = ca.solve(I, tau - ca.cross(omega, ca.mtimes(I, omega)))
        
        return ca.vertcat(pos_dot, euler_dot, vel_dot, omega_dot)
    
    def solve(self, x0, x_ref):
        """
        Solve MPC optimization problem
        
        Args:
            x0: Current state
            x_ref: Reference trajectory (nx × (N+1))
        
        Returns:
            u_opt: Optimal control sequence
            x_pred: Predicted state trajectory
        """
        # Initial guess
        x_init = np.tile(x0.reshape(-1, 1), (1, self.N + 1))
        u_init = np.zeros((self.nu, self.N))
        
        x0_opt = np.concatenate([
            x_init.flatten(),
            u_init.flatten()
        ])
        
        # Parameters
        p = x_ref.flatten()
        
        # Set initial state constraint
        self.lbx[0:self.nx] = x0.tolist()
        self.ubx[0:self.nx] = x0.tolist()
        
        # Solve
        sol = self.solver(
            x0=x0_opt,
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=p
        )
        
        # Extract solution
        x_opt = sol['x']
        
        # Reshape solution
        X_opt = x_opt[:self.nx * (self.N + 1)].reshape((self.nx, self.N + 1))
        U_opt = x_opt[self.nx * (self.N + 1):].reshape((self.nu, self.N))
        
        return U_opt[:, 0], X_opt

class QuadrotorSimulator:
    """Simple quadrotor simulator for testing MPC"""
    
    def __init__(self, params):
        self.params = params
        self.state = np.zeros(12)  # [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]
        
    def reset(self, initial_state=None):
        """Reset simulator state"""
        if initial_state is not None:
            self.state = initial_state.copy()
        else:
            self.state = np.zeros(12)
    
    def step(self, u, dt):
        """Simulate one time step"""
        def dynamics(t, x):
            return self._continuous_dynamics(x, u)
        
        # Integrate dynamics
        sol = solve_ivp(dynamics, [0, dt], self.state, 
                       method='RK45', rtol=1e-6)
        
        self.state = sol.y[:, -1]
        return self.state.copy()
    
    def _continuous_dynamics(self, x, u):
        """Continuous-time quadrotor dynamics"""
        # Extract states
        pos = x[0:3]
        euler = x[3:6]
        vel = x[6:9]
        omega = x[9:12]
        
        # Extract inputs
        T = u[0]
        tau = u[1:4]
        
        # Parameters
        m = self.params['mass']
        g = self.params['gravity']
        I = self.params['inertia']
        
        # Rotation matrix
        phi, theta, psi = euler
        
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_theta, s_theta = np.cos(theta), np.sin(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)
        
        R = np.array([
            [c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi],
            [s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi],
            [-s_theta, c_theta*s_phi, c_theta*c_phi]
        ])
        
        # Position derivatives
        pos_dot = vel
        
        # Euler angle derivatives
        W = np.array([
            [1, s_phi*np.tan(theta), c_phi*np.tan(theta)],
            [0, c_phi, -s_phi],
            [0, s_phi/np.cos(theta), c_phi/np.cos(theta)]
        ])
        
        euler_dot = W @ omega
        
        # Velocity derivatives
        thrust_world = R @ np.array([0, 0, T])
        vel_dot = thrust_world / m + np.array([0, 0, -g])
        
        # Angular velocity derivatives
        omega_dot = np.linalg.solve(I, tau - np.cross(omega, I @ omega))
        
        return np.concatenate([pos_dot, euler_dot, vel_dot, omega_dot])

def generate_trajectory(t_span, trajectory_type='circular'):
    """Generate reference trajectory"""
    if trajectory_type == 'circular':
        radius = 2.0
        height = 2.0
        angular_freq = 0.5
        
        x_ref = radius * np.cos(angular_freq * t_span)
        y_ref = radius * np.sin(angular_freq * t_span)
        z_ref = height * np.ones_like(t_span)
        yaw_ref = np.zeros_like(t_span)
        
        # Velocities
        vx_ref = -radius * angular_freq * np.sin(angular_freq * t_span)
        vy_ref = radius * angular_freq * np.cos(angular_freq * t_span)
        vz_ref = np.zeros_like(t_span)
        
        # Create full reference trajectory
        ref_traj = np.zeros((12, len(t_span)))
        ref_traj[0, :] = x_ref   # x
        ref_traj[1, :] = y_ref   # y
        ref_traj[2, :] = z_ref   # z
        ref_traj[5, :] = yaw_ref # yaw
        ref_traj[6, :] = vx_ref  # vx
        ref_traj[7, :] = vy_ref  # vy
        ref_traj[8, :] = vz_ref  # vz
        
    elif trajectory_type == 'step':
        ref_traj = np.zeros((12, len(t_span)))
        ref_traj[2, :] = 2.0  # Step to 2m height
        
    return ref_traj


def run_mpc_simulation():
	"""Run complete MPC simulation"""
	
	# Simulation parameters
	dt = 0.1
	T_sim = 20.0
	t_span = np.arange(0, T_sim, dt)
	N_steps = len(t_span)
	
	# Initialize MPC controller
	mpc = QuadrotorMPC(dt=dt, N=10)
	
	# Initialize simulator
	simulator = QuadrotorSimulator(mpc.params)
	
	# Generate reference trajectory
	ref_trajectory = generate_trajectory(t_span, 'circular')
	
	# Initialize logging
	states_log = np.zeros((12, N_steps))
	controls_log = np.zeros((4, N_steps))
	computation_times = np.zeros(N_steps)
	
	# Initial state
	x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
	simulator.reset(x0)
	
	print("Starting MPC simulation...")
	
	# Main simulation loop
	for k in range(N_steps):
		start_time = time.time()
		
		# Get current state
		current_state = simulator.state
		states_log[:, k] = current_state
		
		# Get reference for prediction horizon
		if k + mpc.N < N_steps:
			x_ref = ref_trajectory[:, k:k+mpc.N+1]
		else:
			# Extend last reference
			x_ref = np.tile(ref_trajectory[:, -1:], (1, mpc.N+1))
		
		# Solve MPC
		try:
			u_opt, x_pred = mpc.solve(current_state, x_ref)
			controls_log[:, k] = u_opt
		except Exception as e:
			print(f"MPC failed at step {k}: {e}")
			u_opt = np.array([mpc.params['mass'] * mpc.params['gravity'], 0, 0, 0])
			controls_log[:, k] = u_opt
		
		# Record computation time
		computation_times[k] = time.time() - start_time
		
		# Apply control to simulator
		simulator.step(u_opt, dt)
		
		if k % 50 == 0:
			print(f"Step {k}/{N_steps}, computation time: {computation_times[k]:.3f}s")
	
	print("Simulation completed!")
	
	return {
		'time': t_span,
		'states': states_log,
		'controls': controls_log,
		'reference': ref_trajectory,
		'computation_times': computation_times
	}

def plot_results(results):
	"""Plot simulation results"""
	
	t = results['time']
	states = results['states']
	controls = results['controls']
	ref = results['reference']
	comp_times = results['computation_times']
	
	# 3D trajectory plot
	fig = plt.figure(figsize=(15, 12))
	
	# 3D trajectory
	ax1 = fig.add_subplot(2, 3, 1, projection='3d')
	ax1.plot(states[0, :], states[1, :], states[2, :], 'b-', label='Actual', linewidth=2)
	ax1.plot(ref[0, :], ref[1, :], ref[2, :], 'r--', label='Reference', linewidth=2)
	ax1.set_xlabel('X (m)')
	ax1.set_ylabel('Y (m)')
	ax1.set_zlabel('Z (m)')
	ax1.set_title('3D Trajectory')
	ax1.legend()
	ax1.grid(True)
	
	# Position tracking
	ax2 = fig.add_subplot(2, 3, 2)
	ax2.plot(t, states[0, :], 'b-', label='X actual')
	ax2.plot(t, ref[0, :], 'r--', label='X reference')
	ax2.plot(t, states[1, :], 'g-', label='Y actual')
	ax2.plot(t, ref[1, :], 'c--', label='Y reference')
	ax2.plot(t, states[2, :], 'm-', label='Z actual')
	ax2.plot(t, ref[2, :], 'y--', label='Z reference')
	ax2.set_xlabel('Time (s)')
	ax2.set_ylabel('Position (m)')
	ax2.set_title('Position Tracking')
	ax2.legend()
	ax2.grid(True)
	
	# Attitude
	ax3 = fig.add_subplot(2, 3, 3)
	ax3.plot(t, np.rad2deg(states[3, :]), label='Roll')
	ax3.plot(t, np.rad2deg(states[4, :]), label='Pitch')
	ax3.plot(t, np.rad2deg(states[5, :]), label='Yaw')
	ax3.set_xlabel('Time (s)')
	ax3.set_ylabel('Angle (deg)')
	ax3.set_title('Attitude')
	ax3.legend()
	ax3.grid(True)
	
	# Control inputs
	ax4 = fig.add_subplot(2, 3, 4)
	ax4.plot(t, controls[0, :], label='Thrust')
	ax4.set_xlabel('Time (s)')
	ax4.set_ylabel('Thrust (N)')
	ax4.set_title('Thrust Command')
	ax4.grid(True)
	
	ax5 = fig.add_subplot(2, 3, 5)
	ax5.plot(t, controls[1, :], label='τ_φ')
	ax5.plot(t, controls[2, :], label='τ_θ')
	ax5.plot(t, controls[3, :], label='τ_ψ')
	ax5.set_xlabel('Time (s)')
	ax5.set_ylabel('Torque (N⋅m)')
	ax5.set_title('Torque Commands')
	ax5.legend()
	ax5.grid(True)
	
	# Computation times
	ax6 = fig.add_subplot(2, 3, 6)
	ax6.plot(t, comp_times * 1000)
	ax6.set_xlabel('Time (s)')
	ax6.set_ylabel('Computation Time (ms)')
	ax6.set_title('MPC Computation Time')
	ax6.grid(True)
	
	plt.tight_layout()
	plt.show()
	
	# Performance metrics
	print("\n=== PERFORMANCE METRICS ===")
	position_error = np.linalg.norm(states[0:3, :] - ref[0:3, :], axis=0)
	print(f"Mean position error: {np.mean(position_error):.3f} m")
	print(f"Max position error: {np.max(position_error):.3f} m")
	print(f"RMS position error: {np.sqrt(np.mean(position_error**2)):.3f} m")
	print(f"Mean computation time: {np.mean(comp_times)*1000:.1f} ms")
	print(f"Max computation time: {np.max(comp_times)*1000:.1f} ms")

if __name__ == "__main__":
	# Run simulation
	results = run_mpc_simulation()
	
	# Plot results
	plot_results(results)