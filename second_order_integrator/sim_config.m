%% Simulation Parameters
config.t_max = 60; % Maximum simulation time (seconds)
config.timestep = 0.05; % Time step for simulation (seconds)

%% General Configuration (Shared Among Agents)
config.geometric_tol = 0.1; % Margin to increase objects to avoid numerical errors
config.R_circle = 5; % Radius of the arena
config.R_agent = 0.5; % Radii of the agents
config.p0_std = 0.005; % Small variation in the initial position of the agents

config.v_mod = 1; % Preferred velocity magnitude
config.v_max = 2; % Maximum velocity
config.u_max = 1; % Maximum acceleration (m/s²)

%% Control Barrier Function (CBF) Parameters
% Same nomenclature as in the paper (https://arxiv.org/abs/2409.10117)
config.alpha_vo = 100;
config.alpha_c = 10;

config.k_u = 1; % Control input scaling
config.k_vo = 1e3; % Scaling for cone constraints

%% PID Controller Parameters
config.P = 1; % Proportional gain
config.D = 0.5; % Derivative gain

%% Goal Detection Parameters
config.goal_reached_tol = 0.5;

%% Environment Configuration
config.consider_only_min_t_col = false; % Only consider minimum Time-To-Collision instead of T_col to every agent
config.include_VO_as_hard_constraint = false; % Treat VO as a hard constraint without slack variables
config.max_iterations_fail = 3; % Max iterations before failure handling
config.use_safe_cbf = true; % Whether to include the safe-cbf as constraints or not

%% Camera Settings
cameraTranslation = [0, 0, 20]; % Camera position
cameraRotation = [0, 90, 90]; % Camera orientation
