% =========================================================================
% STARTUP: SLOW RAMP STEER (RMP) TESTING
% Method: Quasi-Steady-State Maneuver for Agility/Gain Extraction
% =========================================================================
clc; close all;
disp('--- Initializing Ramp Steer Trajectory and Parameters ---');

% -------------------------------------------------------------------------
% 1. VEHICLE PARAMETERS
% -------------------------------------------------------------------------
disp('-> Loading Vehicle Parameters...');
veh_params.m = 1600; veh_params.Iz = 2600; veh_params.Iw = 1.2;
veh_params.lf = 1.3; veh_params.lr = 1.3; veh_params.L = 2.6; 
veh_params.tw = 1.65; veh_params.h_cg = 0.35; veh_params.Rw = 0.34;
veh_params.rho = 1.225; veh_params.CdA = 0.6; veh_params.g = 9.81;
veh_params.roll_bias_f = 0.55; veh_params.roll_bias_r = 0.45;
veh_params.brake_bias_f = 0.65;

% Steering & Tires
veh_params.SR = 540 / 35; % Steering Ratio (Handwheel to Roadwheel)
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;
veh_params.mu_max = 1.2; veh_params.C_rr = 0.015; 
veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;

% Powertrain & Thermal
veh_params.G = 10; % Powertrain Reduction Gear Ratio
veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; veh_params.P_regen_max_kw = 250;
veh_params.V_pack = 3.6 * 238; 
veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;
veh_params.m_batt = 450;  veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 250; veh_params.T_coolant_batt = 20;
veh_params.m_motor = 80;  veh_params.cp_motor = 900; 
veh_params.hA_cool_motor = 350; veh_params.T_ambient = 30;

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
% 3. PLANNER: ULTRA-SLOW RAMP & VELOCITY PROFILE
% -------------------------------------------------------------------------
disp('-> Generating Ramp Steer Input (0.5 deg/s at Handwheel)...');
sim_params.dt = 0.001;
t_end_sim = 60.0; % Extended to 60s to accommodate the ultra-slow ramp
t_sim = (0:sim_params.dt:t_end_sim)';

% Constant Velocity Profile (100 km/h)
u_ms = 100.0 / 3.6; 
vx_target_array = timeseries(u_ms * ones(size(t_sim)), t_sim);

% Ramp Parameters
t_start = 15.0;            
target_handwheel_rate_deg_s = 0.5; % ULTRA-SLOW to prevent controller wake-up
ramp_rate_deg_s = target_handwheel_rate_deg_s / veh_params.SR; % ~0.0324 deg/s at road wheels

steer_rad = zeros(size(t_sim));
for i = 1:length(t_sim)
    t = t_sim(i);
    if t >= t_start
        steer_rad(i) = (ramp_rate_deg_s * (t - t_start)) * (pi/180);
    end
end

steer_profile = timeseries(steer_rad, t_sim);

% -------------------------------------------------------------------------
% 4. DUMMY TRACK DATA & REFERENCES (CRASH PREVENTION)
% -------------------------------------------------------------------------
X_track = [0; 2000]; Y_track = [0; 0];
s_track = [0; 2000]; psi_track = [0; 0]; kappa_track = [0; 0];
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
disp('--- Setup Complete: Vehicle is ready for Quasi-Steady Ramp Steer. ---');