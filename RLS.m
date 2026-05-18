% =========================================================================
% STARTUP: STEERING RELEASE (RLS) TESTING
% Method: Steady-State Snap-Release for Damping Ratio Extraction
% =========================================================================
clc; close all;
disp('--- Initializing Steering Release Trajectory and Parameters ---');

% -------------------------------------------------------------------------
% 1. VEHICLE PARAMETERS (Unified & Complete for Simulink Coder)
% -------------------------------------------------------------------------
disp('-> Loading Vehicle Parameters...');

% --- Chassis & Inertia ---
veh_params.m = 1600; 
veh_params.Iz = 2600; 
veh_params.Iw = 1.2;
veh_params.lf = 1.3; 
veh_params.lr = 1.3; 
veh_params.L = 2.6; 
veh_params.tw = 1.65; 
veh_params.h_cg = 0.35; 
veh_params.Rw = 0.34;

% --- Aerodynamics & Environment ---
veh_params.rho = 1.225; 
veh_params.CdA = 0.6; 
veh_params.g = 9.81;
veh_params.T_ambient = 30;

% --- Steering, Roll, & Braking ---
veh_params.SR = 540 / 35;       % Steering Ratio
veh_params.roll_bias_f = 0.55; 
veh_params.roll_bias_r = 0.45;
veh_params.brake_bias_f = 0.65;

% --- Tire Mechanics ---
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;
veh_params.mu_max = 1.2; 
veh_params.C_rr = 0.015; 
veh_params.C_tire = 1.3; 
veh_params.E_tire = -1.0;

% --- Powertrain ---
veh_params.G = 10;              % Reduction Gear Ratio
veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; 
veh_params.P_regen_max_kw = 250;

% --- Battery Electrics & Thermal ---
veh_params.V_pack = 3.6 * 238; 
veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;
veh_params.m_batt = 450;  
veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 250; 
veh_params.T_coolant_batt = 20;

% --- Motor Thermal ---
veh_params.m_motor = 80;  
veh_params.cp_motor = 900; 
veh_params.hA_cool_motor = 350; 

% -------------------------------------------------------------------------
% 2. LQR GAIN SCHEDULING
% -------------------------------------------------------------------------
disp('-> Calculating LQR Gains...');
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
% 3. PLANNER: THE STEADY-STATE "SNAP" PROFILE
% -------------------------------------------------------------------------
disp('-> Generating Steering Release Input (With 10s Longitudinal Buffer)...');
sim_params.dt = 0.001;
t_end_sim = 40.0; % Extended to 40s total
t_sim = (0:sim_params.dt:t_end_sim)';

% Constant Velocity Profile (100 km/h)
u_ms = 100.0 / 3.6; 
vx_target_array = timeseries(u_ms * ones(size(t_sim)), t_sim);

% Release Parameters (Delayed Timeline)
steer_ss_deg = 1.5;  % Generates roughly 0.45G at 100km/h
t_ramp_start = 10.0; % 10 full seconds of straight-line driving for controller to settle
t_ramp_end   = 13.0; % 3-second gentle turn-in
t_release    = 25.0; % 12 seconds of pure steady-state cornering before snap

steer_rad = zeros(size(t_sim));
for i = 1:length(t_sim)
    t = t_sim(i);
    if t >= t_ramp_start && t < t_ramp_end
        % Build up to steady state
        steer_rad(i) = steer_ss_deg * ((t - t_ramp_start) / (t_ramp_end - t_ramp_start)) * (pi/180);
    elseif t >= t_ramp_end && t < t_release
        % Hold steady state cornering
        steer_rad(i) = steer_ss_deg * (pi/180);
    elseif t >= t_release && t < (t_release + 0.1)
        % The "Snap" release (returns to 0 in 0.1s)
        steer_rad(i) = steer_ss_deg * (1.0 - ((t - t_release) / 0.1)) * (pi/180);
    else
        % Hands off the wheel
        steer_rad(i) = 0.0;
    end
end

steer_profile = timeseries(steer_rad, t_sim);

% -------------------------------------------------------------------------
% 4. DUMMY TRACK DATA & REFERENCES (CRASH PREVENTION)
% -------------------------------------------------------------------------
X_track = [0; 2000]; Y_track = [0; 0];
assignin('base', 'X_track', X_track); assignin('base', 'Y_track', Y_track);
kappa_ref_array = timeseries(zeros(size(t_sim)), t_sim);
psi_ref_array   = timeseries(zeros(size(t_sim)), t_sim);

% -------------------------------------------------------------------------
% 5. SIMULATION INITIAL CONDITIONS
% -------------------------------------------------------------------------
sim_params.t_end = t_end_sim; 
sim_params.initial_speed = 0.0; 
sim_params.initial_X = 0; 
sim_params.initial_Y = 0; 
sim_params.initial_psi = 0;
disp('--- Setup Complete: Vehicle is ready for Steering Release. ---');