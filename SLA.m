% =========================================================================
% REWRITTEN STARTUP: TRANSIENT SLALOM TESTING
% Method: 18m Standard Cone Slalom at Constant Velocity
% =========================================================================
clc; close all;
disp('--- Initializing Slalom Trajectory and Parameters ---');

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

% Specific Tire Stiffness mapping
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;
veh_params.mu_max = 1.2; veh_params.C_rr = 0.015; 
veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;

% Powertrain & Battery Limits (238S 3P Configuration)
veh_params.G = 10; veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; veh_params.P_regen_max_kw = 250;
veh_params.V_pack = 3.6 * 238; 
veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;

% Thermal Constants
veh_params.m_batt = 450;  veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 250; veh_params.T_coolant_batt = 20;
veh_params.m_motor = 80;  veh_params.cp_motor = 900; 
veh_params.hA_cool_motor = 350; veh_params.T_ambient = 30;

% -------------------------------------------------------------------------
% 2. LQR GAIN SCHEDULING
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
% 3. PLANNER: SPATIAL TRACK (18m SLALOM) & VELOCITY PROFILE
% -------------------------------------------------------------------------
disp('-> Generating Spatial Track (18m Slalom)...');
runway_length = 100.0; % Increased to 100m straight to accelerate and stabilize
cone_spacing = 18.0;   % Standard 18m distance between cones
num_cones = 12;
slalom_length = num_cones * cone_spacing;

X_track = linspace(0, runway_length + slalom_length + 40, 5000)';
Y_track = zeros(size(X_track));

% Generate Sinusoidal Weave through cones starting exactly after the runway
slalom_idx = (X_track > runway_length) & (X_track <= runway_length + slalom_length);
Y_track(slalom_idx) = 2.0 * sin((pi/cone_spacing) * (X_track(slalom_idx) - runway_length));

dx = diff(X_track); dy = diff(Y_track);
s_track = [0; cumsum(sqrt(dx.^2 + dy.^2))];

% --- VELOCITY PROFILE ---
u_ms = 72.0 / 3.6; % Target 72 km/h (20 m/s)
V_profile = u_ms * ones(size(s_track)); 

% --- Path Metadata ---
dy_ds = gradient(Y_track) ./ gradient(s_track);
dx_ds = gradient(X_track) ./ gradient(s_track);
psi_track = atan2(dy_ds, dx_ds);
dpsi_ds = gradient(psi_track) ./ gradient(s_track);
kappa_track = dpsi_ds;

% -------------------------------------------------------------------------
% 4. INITIAL CONDITIONS & SIMULATION TIMING
% -------------------------------------------------------------------------
% Pad the total simulation time to account for the slower average speed during initial acceleration
sim_params.t_end = (runway_length + slalom_length + 40) / (u_ms * 0.75) + 3.0; 
sim_params.dt = 0.001;
sim_params.initial_speed = 0.0; % Start from a standstill
sim_params.initial_X = 0; 
sim_params.initial_Y = 0; 
sim_params.initial_psi = 0;

disp('--- Setup Complete: Vehicle is ready for slalom. ---');