% =========================================================================
% STARTUP: STEERING PULSE (STP) TESTING
% Method: Broadband Impulse Excitation for Natural Frequency Extraction
% =========================================================================
clc; close all;
disp('--- Initializing Steering Pulse Trajectory and Parameters ---');

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

% Tire Stiffness 
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;
veh_params.mu_max = 1.2; veh_params.C_rr = 0.015; 
veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;

% Powertrain & Thermal
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
% 3. PLANNER: DISCRETE TRIANGLE PULSE & VELOCITY PROFILE
% -------------------------------------------------------------------------
disp('-> Generating Steering Pulse Input...');
sim_params.dt = 0.001;
t_end_sim = 25.0; % 15s settle + 10s ring-down analysis
t_sim = (0:sim_params.dt:t_end_sim)';

% Constant Velocity Profile (100 km/h)
u_ms = 100.0 / 3.6; 
vx_target_array = [t_sim, u_ms * ones(size(t_sim))];

% Triangle Pulse Parameters
t_start = 15.0;            % Start of pulse
t_peak  = 15.2;            % Apex of the flick (0.2s ramp up)
t_end   = 15.4;            % Return to center (0.2s ramp down)
steer_amplitude_deg = 3.0; % Amplitude of the pulse

steer_rad = zeros(size(t_sim));
for i = 1:length(t_sim)
    t = t_sim(i);
    if t >= t_start && t < t_peak
        % Ramp up
        steer_rad(i) = (steer_amplitude_deg * (t - t_start) / (t_peak - t_start)) * (pi/180);
    elseif t >= t_peak && t <= t_end
        % Ramp down
        steer_rad(i) = (steer_amplitude_deg * (t_end - t) / (t_end - t_peak)) * (pi/180);
    else
        steer_rad(i) = 0.0;
    end
end

steer_profile = [t_sim, steer_rad];

% -------------------------------------------------------------------------
% 4. DUMMY TRACK DATA & INITIAL CONDITIONS
% -------------------------------------------------------------------------
X_track = [0; 2000]; Y_track = [0; 0];
s_track = [0; 2000]; psi_track = [0; 0]; kappa_track = [0; 0];
assignin('base', 'X_track', X_track); assignin('base', 'Y_track', Y_track);

sim_params.t_end = t_end_sim; 
sim_params.initial_speed = 0.0; 
sim_params.initial_X = 0; 
sim_params.initial_Y = 0; 
sim_params.initial_psi = 0;
disp('--- Setup Complete: Vehicle is ready for Steering Pulse. ---');