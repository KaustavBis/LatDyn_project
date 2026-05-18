% =========================================================================
% REWRITTEN STARTUP: ULTRA-SMOOTH QUASI-STEADY ACCELERATION
% Method: Time-to-Space Velocity Mapping (ISO 4138 Quasi-Static)
% =========================================================================
clc; close all;
disp('--- Initializing Ultra-Smooth ISO 4138 Velocity Profile ---');

% -------------------------------------------------------------------------
% 1. VEHICLE PARAMETERS (Friday Baseline)
% -------------------------------------------------------------------------
disp('-> Loading Vehicle Parameters...');
veh_params.m = 1600; veh_params.Iz = 2600; veh_params.Iw = 1.2;
veh_params.lf = 1.3; veh_params.lr = 1.3; veh_params.L = 2.6; 
veh_params.tw = 1.65; veh_params.h_cg = 0.35; veh_params.Rw = 0.34;
veh_params.rho = 1.225; veh_params.CdA = 0.6; veh_params.g = 9.81;
veh_params.roll_bias_f = 0.55; veh_params.roll_bias_r = 0.45;
veh_params.brake_bias_f = 0.65;

% Specific Tire Stiffness mapping
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;
veh_params.mu_max = 1.2; veh_params.C_rr = 0.015; 
veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;

% Powertrain & Battery Limits
veh_params.G = 10; veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; veh_params.P_regen_max_kw = 250;
veh_params.V_pack = 3.6 * 238; veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;

% Thermal Constants
veh_params.m_batt = 450;  veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 250; veh_params.T_coolant_batt = 20;
veh_params.m_motor = 80;  veh_params.cp_motor = 900; 
veh_params.hA_cool_motor = 350; veh_params.T_ambient = 30;

% -------------------------------------------------------------------------
% 2. LQR GAIN SCHEDULING
% -------------------------------------------------------------------------
% --- THE NEW COST FUNCTION ---
Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]);

% R: Penalty on Control Effort (Mz).
R_lqr = 1/(2500^2);

V_op = 5:5:85; K_scheduled = zeros(length(V_op), 2); 
for i = 1:length(V_op)
    vx_t = V_op(i);
    A_lin = [-(veh_params.ks_f + veh_params.ks_r)/(veh_params.m*vx_t), ((veh_params.ks_r*veh_params.lr - veh_params.ks_f*veh_params.lf)/(veh_params.m*vx_t)) - vx_t;
             (veh_params.ks_r*veh_params.lr - veh_params.ks_f*veh_params.lf)/(veh_params.Iz*vx_t), -(veh_params.ks_f*veh_params.lf^2 + veh_params.ks_r*veh_params.lr^2)/(veh_params.Iz*vx_t)];
    B_lin = [veh_params.ks_f/veh_params.m, 0; (veh_params.lf*veh_params.ks_f)/veh_params.Iz, 1/veh_params.Iz];
    try, K_scheduled(i, :) = lqr(A_lin, B_lin(:,2), Q_lqr, R_lqr); catch, if i > 1, K_scheduled(i, :) = K_scheduled(i-1, :); end; end
end

% -------------------------------------------------------------------------
% 3. PLANNER: SPATIAL TRACK & SMOOTH VELOCITY RAMP
% -------------------------------------------------------------------------
disp('-> Generating Spatial Track & Ultra-Slow Velocity Ramp...');
runway_length = 50.0; % 50m to stabilize at 20 km/h
R_track = 50.0;

% Build X/Y Coordinates (40 Laps)
theta_circ = linspace(-pi/2, 80*pi - pi/2, 15000)'; 
X_track = [linspace(0, runway_length, 200)'; runway_length + R_track * cos(theta_circ(2:end))];
Y_track = [zeros(200, 1); R_track + R_track * sin(theta_circ(2:end))];

% Distance Vector
dx = diff(X_track); dy = diff(Y_track);
s_track = [0; cumsum(sqrt(dx.^2 + dy.^2))];

% --- SIMPLIFIED VELOCITY MAPPING ---
u_ms = 20.0 / 3.6;          % Initial speed (5.55 m/s)
a_target = 0.2;            % True Quasi-Static ISO Rate (0.05 m/s^2)

% Vectorized kinematic equation: v = sqrt(u^2 + 2*a*distance)
% max(0, ...) ensures the 50m runway distance evaluates to 0 acceleration.
V_profile = sqrt(u_ms^2 + 2 * a_target * max(0, s_track - runway_length));

% Safety Cap
V_profile = min(V_profile, 180/3.6);

% Apply a base MATLAB moving average to round off the mathematical corner at 50m
% (Eliminates the infinite jerk spike when transitioning from 0 accel to 0.05 accel)
window_size = 300; 
V_profile = conv(V_profile, ones(window_size,1)/window_size, 'same');
% Ensure the start of the runway stays exactly at target speed after convolution
V_profile(1:100) = u_ms; 

% --- Path Metadata ---
kappa_track = [zeros(200,1); ones(length(theta_circ)-1, 1)/R_track];
psi_track = [zeros(200,1); theta_circ(2:end) + pi/2];

% -------------------------------------------------------------------------
% 4. INITIAL CONDITIONS & SIMULATION TIMING
% -------------------------------------------------------------------------
% At 0.05 m/s^2, it takes a long time to reach the friction limit.
sim_params.t_end = 400.0; 
sim_params.dt = 0.001;

sim_params.initial_speed = u_ms;
sim_params.initial_X = 0; 
sim_params.initial_Y = 0; 
sim_params.initial_psi = 0;
disp('--- Setup Complete: Vehicle is ready for quasi-static sweep. ---');