% =========================================================================
% MASTER SIMULATION INITIALIZATION: ISO 3888-1 DOUBLE LANE CHANGE (DLC)
% Execution: Run this script in the base workspace BEFORE opening Simulink.
% Purpose: Transient handling evaluation and LQR Step-Response Tuning.
% =========================================================================

clc; close all;
disp('--- Initializing DLC Test Environment ---');

% =========================================================================
% 1. VEHICLE PARAMETERS (Centralized Struct Architecture)
% =========================================================================
disp('-> Loading Vehicle Parameters...');
veh_params.m = 1600; veh_params.Iz = 2600; veh_params.Iw = 1.2;
veh_params.lf = 1.3; veh_params.lr = 1.3; veh_params.L = 2.6; 
veh_params.tw = 1.65; veh_params.h_cg = 0.35; veh_params.Rw = 0.34;
veh_params.rho = 1.225; veh_params.CdA = 0.6; veh_params.g = 9.81;
veh_params.roll_bias_f = 0.55; veh_params.roll_bias_r = 0.45;
veh_params.brake_bias_f = 0.65;

% Your specific DLC Tire Stiffness mapping
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;
veh_params.mu_max = 1.2; veh_params.C_rr = 0.015; 
veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;

veh_params.G = 10; veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; veh_params.P_regen_max_kw = 250;
veh_params.V_pack = 3.6 * 238; veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;

% Thermal Constants (Required so the Plant doesn't throw dimension errors)
veh_params.m_batt = 450;  veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 250; veh_params.T_coolant_batt = 20;
veh_params.m_motor = 80;  veh_params.cp_motor = 900; 
veh_params.hA_cool_motor = 350; veh_params.T_ambient = 30;

% =========================================================================
% 2. TRACK ENVIRONMENT: ISO 3888-1 DOUBLE LANE CHANGE
% =========================================================================
disp('-> Generating ISO DLC Track Coordinates...');

track_params.mu_modifier = 1.5; 
X_track = 0:0.1:500; % 500m long track, 10cm resolution
Y_track = zeros(size(X_track));

% ISO 3888-1 Standard Dimensions (Corrected)
L1 = 50.0;     % Length of first lane change (m)
L_hold = 30.0; % Length of stabilization phase in left lane (m)
L2 = 50.0;     % Length of second lane change back to right (m)
W  = 30;      % Standard single-lane lateral offset (m)

% Track Phase Markers
x_start_1 = 400;
x_start_hold = x_start_1 + L1;
x_start_2 = x_start_hold + L_hold;
x_end_2 = x_start_2 + L2;

% --- EXTENSION LOGIC ---
% Calculate total track length to be exactly 1000m after the lane change completes
total_track_length = x_end_2 + 1000.0; 
X_track = 0:0.1:total_track_length; 
Y_track = zeros(size(X_track));

% Generate smooth Haversine trajectory
for i = 1:length(X_track)
    x = X_track(i);
    if x >= x_start_1 && x < x_start_hold
        Y_track(i) = (W/2) * (1 - cos(pi * (x - x_start_1) / L1));
    elseif x >= x_start_hold && x < x_start_2
        Y_track(i) = W;
    elseif x >= x_start_2 && x < x_end_2
        Y_track(i) = (W/2) * (1 + cos(pi * (x - x_start_2) / L2));
    elseif x >= x_end_2
        Y_track(i) = 0;
    end
end

% Pack coordinates into struct
track_params.X = X_track';
track_params.Y = Y_track';

% Calculate Arc Length (s)
dX = diff(track_params.X); dY = diff(track_params.Y);
track_params.s = [0; cumsum(sqrt(dX.^2 + dY.^2))];

% Differential Geometry (Curvature and Heading)
dx_ds = gradient(track_params.X, track_params.s);
dy_ds = gradient(track_params.Y, track_params.s);
ddx_ds = gradient(dx_ds, track_params.s);
ddy_ds = gradient(dy_ds, track_params.s);

track_params.psi = wrapToPi(atan2(dy_ds, dx_ds));
kappa_raw = (dx_ds.*ddy_ds - dy_ds.*ddx_ds) ./ max((dx_ds.^2 + dy_ds.^2).^(3/2), 1e-6);
kappa_track = movmean(kappa_raw, 50); 
kappa_track(abs(kappa_track) < 0.00125) = 0; % Remove microscopic numerical noise
track_params.kappa = kappa_track;

% --- CONSTANT TEST SPEED PROFILE ---
% if exist('V_test', 'var')
%     % Inherit the value and convert back to km/h for the profile logic
%     test_speed_kmh = V_test * 3.6; 
%     disp(['-> Init: Master Script detected. Profiling for ', num2str(test_speed_kmh), ' km/h.']);
% else
    % Fallback: If you run this script manually without the Master Script
    test_speed_kmh = 72; 
    V_test = test_speed_kmh / 3.6; % Create V_test so downstream math doesn't break
    disp(['-> Init: Standalone mode. Using default speed: ', num2str(test_speed_kmh), ' km/h.']);
% end
track_params.vx = ones(length(track_params.s), 1) * (test_speed_kmh / 3.6);
track_params.length_m = max(track_params.s);

% =========================================================================
% 3. PLANNER TUNING
% =========================================================================
disp('-> Initializing Trajectory Planner...');

planner_params.lookahead_min = 10.0;  % Increased to stop oscillation
planner_params.lookahead_max = 25.0; 
planner_params.lookahead_gain = 0.4;  % Gives the planner more vision at speed
planner_params.tube_width_m = 0;

% =========================================================================
% 4. LQR GAIN SCHEDULING (Mapped to veh_params)
% =========================================================================
disp('-> Solving Algebraic Riccati Equations...');
% Q_lqr = diag([1/(0.1^2), 1/(0.001)]); % [vy penalty, r penalty]
% R_lqr = 1/(15000^2);                   % Control effort penalty (Mz_cmd limit)   
Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]);

% R: Penalty on Control Effort (Mz).
R_lqr = 1/(2500^2);
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
% 5. SIMULATION SETTINGS (Spawn Coordinates)
% =========================================================================
sim_params.dt = 0.001;               
sim_params.t_end = 30.0; % DLC is much shorter than Silverstone          
sim_params.initial_speed = track_params.vx(1); 
sim_params.initial_X = track_params.X(1);
sim_params.initial_Y = track_params.Y(1);
sim_params.initial_psi = track_params.psi(1);

disp(['--- Initialization Complete: DLC Track ready at ', num2str(test_speed_kmh), ' km/h. ---']);