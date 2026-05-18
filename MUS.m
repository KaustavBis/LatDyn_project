% =========================================================================
% STARTUP: MU-SPLIT DISTURBANCE TESTING (MUS - ABS BRAKING)
% Method: High-speed acceleration followed by hard braking into a mu-split
% =========================================================================
clc; close all;
disp('--- Initializing Mu-Split ABS Braking Parameters ---');

% -------------------------------------------------------------------------
% 1. VEHICLE PARAMETERS
% -------------------------------------------------------------------------
veh_params.m = 1600; veh_params.Iz = 2600; veh_params.Iw = 1.2;
veh_params.lf = 1.3; veh_params.lr = 1.3; veh_params.L = 2.6; 
veh_params.tw = 1.65; veh_params.h_cg = 0.35; veh_params.Rw = 0.34;
veh_params.rho = 1.225; veh_params.CdA = 0.6; veh_params.g = 9.81;
veh_params.roll_bias_f = 0.55; veh_params.roll_bias_r = 0.45;
veh_params.brake_bias_f = 0.65;

% Base Tire Stiffness 
veh_params.ks_f = 345000; veh_params.ks_r = 410000;
veh_params.C_rr = 0.015; veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;

% Powertrain & Thermal (3P 238S Configuration)
veh_params.G = 10; veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; veh_params.P_regen_max_kw = 250;
veh_params.V_pack = 3.6 * 238; 
veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;
veh_params.m_batt = 450;  veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 250; veh_params.T_coolant_batt = 20;
veh_params.m_motor = 80;  veh_params.cp_motor = 900; 
veh_params.hA_cool_motor = 350; veh_params.T_ambient = 30;

% -------------------------------------------------------------------------
% 2. LQR GAIN SCHEDULING (Relaxed for Stability)
% -------------------------------------------------------------------------
Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]);
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
% 3. PLANNER: ACCELERATE TO 150 KMPH -> HARD BRAKING -> MU JUMP 
% -------------------------------------------------------------------------
sim_params.dt = 0.001;
sim_params.t_end = 35.0; % Increased duration to fit the full maneuver
t_sim = (0:sim_params.dt:sim_params.t_end)';

% Driver Input: Hands off the wheel (0 deg) to observe pure instability
steer_rad = zeros(size(t_sim));
steer_profile = [t_sim, steer_rad];

% Speed Profile: Accelerate to 150 km/h, then slam the brakes at t = 15s
u_high_ms = 150.0 / 3.6; 
t_brake_start = 15.0; 

vx_target = zeros(size(t_sim));
for i = 1:length(t_sim)
    if t_sim(i) < t_brake_start
        vx_target(i) = u_high_ms; % Full throttle up to 150 km/h
    else
        vx_target(i) = 0.0;       % Hard brake to 0 km/h
    end
end
vx_target_array = [t_sim, vx_target];

% Environmental Disturbance: Hit the ice patch IN THE MIDDLE of braking
mu_high = 1.0; % Dry Tarmac
mu_low  = 0.2; % Ice patch
mu_left = mu_high * ones(size(t_sim));  
mu_right = mu_high * ones(size(t_sim)); 

% Timing: Braking starts at 15.0s, hit the ice patch at 16.5s
t_drop = 16.5;     
t_recover = 25.0; % Stay on ice until the vehicle has likely stopped

% Create the extreme friction profile
for i = 1:length(t_sim)
    if t_sim(i) >= t_drop && t_sim(i) < t_recover
        mu_right(i) = mu_low; 
    else
        mu_right(i) = mu_high; 
    end
end
mu_profile_left = [t_sim, mu_left];
mu_profile_right = [t_sim, mu_right];

% Dummy spatial track to satisfy Scopes
X_track = [0; 2000]; Y_track = [0; 0]; s_track = [0; 2000]; 
psi_track = [0; 0]; kappa_track = [0; 0];
assignin('base', 'X_track', X_track); assignin('base', 'Y_track', Y_track);

% Initial Conditions
sim_params.initial_speed = 0.1; % Start near zero
sim_params.initial_X = 0; sim_params.initial_Y = 0; sim_params.initial_psi = 0;

disp('--- Setup Complete: Ready for ABS Braking Mu-Split. ---');