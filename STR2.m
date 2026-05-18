% =========================================================================
% MASTER SIMULATION INITIALIZATION: ISO 7401 STEP STEER MANEUVER
% Models: STR_2a (Dual Motor / TV) vs STR_2b (Single Motor / Open Diff)
% Execution: Run this script BEFORE opening Simulink or via Master Script.
% Purpose: Transient response evaluation and steady-state understeer analysis.
% =========================================================================
clc; close all;
disp('--- Initializing Step Steer Test Environment ---');

% =========================================================================
% 1. TEST SPEED HANDSHAKE
% =========================================================================
if exist('V_test', 'var')
    test_speed_kmh = V_test * 3.6; 
    disp(['-> Init: Master Script detected. Target cruise speed: ', num2str(test_speed_kmh), ' km/h.']);
else
    test_speed_kmh = 100; % Defaulted to high-speed testing
    V_test = test_speed_kmh / 3.6; 
    disp(['-> Init: Standalone mode. Target cruise speed: ', num2str(test_speed_kmh), ' km/h.']);
end

% =========================================================================
% 2. VEHICLE PARAMETERS (Centralized Struct Architecture)
% =========================================================================
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

% =========================================================================
% 3. LQR GAIN SCHEDULING
% =========================================================================
disp('-> Solving Algebraic Riccati Equations for TV Controller...');

% Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]); %Jittery Calibration
% 
% % R: Penalty on Control Effort (Mz).
% R_lqr = 1/(16000^2); %Jittery Calibration

R_lqr = 1/(2500^2);
Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]);
V_op = 5:5:85; 
K_scheduled = zeros(length(V_op), 2); 

for i = 1:length(V_op)
    vx_t = V_op(i);
    % Linearized Bicycle Model using struct values
    A_lin = [-(veh_params.ks_f + veh_params.ks_r)/(veh_params.m*vx_t), ((veh_params.ks_r*veh_params.lr - veh_params.ks_f*veh_params.lf)/(veh_params.m*vx_t)) - vx_t;
             (veh_params.ks_r*veh_params.lr - veh_params.ks_f*veh_params.lf)/(veh_params.Iz*vx_t), -(veh_params.ks_f*veh_params.lf^2 + veh_params.ks_r*veh_params.lr^2)/(veh_params.Iz*vx_t)];
    B_lin = [veh_params.ks_f/veh_params.m, 0; 
             (veh_params.lf*veh_params.ks_f)/veh_params.Iz, 1/veh_params.Iz];
    try
        K_scheduled(i, :) = lqr(A_lin, B_lin(:,2), Q_lqr, R_lqr);
    catch
        if i > 1, K_scheduled(i, :) = K_scheduled(i-1, :); end
    end
end

% =========================================================================
% 4. HIGH-SPEED LAUNCH & STEP STEER PROFILE GENERATOR
% =========================================================================
disp('-> Generating High-Speed Launch & Ramp-Step Trajectory...');

% --- Timing Parameters ---
% Extended to 25s to allow for a 0-200 km/h launch, settling, and maneuver
sim_params.t_end = 25.0;  
sim_params.dt = 0.001;
t_sim = (0:sim_params.dt:sim_params.t_end)';

% --- Step Steer Parameters ---
steer_amplitude_deg = 2.5; % Target road wheel angle
steering_rate_deg_per_sec = 25.0; % Actuator speed limit

% **UPDATED: Give the car 12 seconds to hit 200 km/h and stabilize**
steer_start_time = 12.0;    

% Build the Steering Array
delta_cmd_deg = zeros(length(t_sim), 1);
steer_duration = steer_amplitude_deg / steering_rate_deg_per_sec;
steer_end_time = steer_start_time + steer_duration;

for i = 1:length(t_sim)
    if t_sim(i) < steer_start_time
        delta_cmd_deg(i) = 0; % Flat out straight line acceleration
    elseif t_sim(i) >= steer_start_time && t_sim(i) <= steer_end_time
        delta_cmd_deg(i) = steering_rate_deg_per_sec * (t_sim(i) - steer_start_time); % Ramp
    else
        delta_cmd_deg(i) = steer_amplitude_deg; % Steady State Hold
    end
end

% Package Inputs into Simulink Timeseries
delta_cmd_rad = delta_cmd_deg * (pi/180);
steer_profile = timeseries(delta_cmd_rad, t_sim);

% The target velocity remains constant. The longitudinal controller will 
% see the massive error at t=0 and apply full throttle until V_test is met.
vx_target_array = timeseries(ones(size(t_sim)) * V_test, t_sim);

% --- DUMMY PATH REFERENCES (To satisfy Simulink interfaces) ---
kappa_ref_array = timeseries(zeros(size(t_sim)), t_sim);
psi_ref_array   = timeseries(zeros(size(t_sim)), t_sim);

% Dummy variables to prevent the Simulink XY Plotter from crashing
X_track = [0; 2000]; % Extended straight line for the plotter
Y_track = [0; 0];


% =========================================================================
% 5. SIMULATION INITIAL CONDITIONS
% =========================================================================
% **UPDATED: Spawn at a complete standstill to allow natural acceleration**
sim_params.initial_speed = 0.0; 
sim_params.initial_X = 0;
sim_params.initial_Y = 0;
sim_params.initial_psi = 0;

disp(['--- Initialization Complete: Ready for Standstill Launch to ', num2str(test_speed_kmh), ' km/h. ---']);