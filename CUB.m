% =========================================================================
% STARTUP: CORNERING UNDER BRAKING (CUB)
% Scenario: Steady-state 150m radius corner at 120 km/h
%           followed by a trail-braking event at t = 15 s
% Purpose:  Validate friction-circle-aware regen allocation (Patent #9)
%           Compare: QP allocator vs existing flat-margin allocator
% =========================================================================
clc; close all;
disp('--- Initializing Cornering Under Braking (CUB) Parameters ---');

% -------------------------------------------------------------------------
% 1. VEHICLE PARAMETERS (identical to DLC / MUS setup)
% -------------------------------------------------------------------------
veh_params.m            = 1600;
veh_params.Iz           = 2600;
veh_params.Iw           = 1.2;
veh_params.lf           = 1.3;
veh_params.lr           = 1.3;
veh_params.L            = 2.6;
veh_params.tw           = 1.65;
veh_params.h_cg         = 0.35;
veh_params.Rw           = 0.34;
veh_params.rho          = 1.225;
veh_params.CdA          = 0.6;
veh_params.g            = 9.81;
veh_params.roll_bias_f  = 0.55;
veh_params.roll_bias_r  = 0.45;
veh_params.brake_bias_f = 0.65;
veh_params.mu_max       = 1.2;

% Tyre stiffness
veh_params.ks_f         = 345000;
veh_params.ks_r         = 410000;
veh_params.C_rr         = 0.015;
veh_params.C_tire       = 1.3;
veh_params.E_tire       = -1.0;

% Powertrain & Thermal (3P 238S — same as DLC/MUS)
veh_params.G            = 10;
veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw  = 500;
veh_params.P_regen_max_kw = 250;
veh_params.V_pack       = 3.6 * 238;
veh_params.Cap_Ah       = (80 * 1000) / veh_params.V_pack;
veh_params.R_pack       = 0.002 * 238 / 3;
veh_params.m_batt       = 450;   veh_params.cp_batt      = 900;
veh_params.hA_cool_batt = 250;   veh_params.T_coolant_batt = 20;
veh_params.m_motor      = 80;    veh_params.cp_motor      = 500;
veh_params.hA_cool_motor = 350;  veh_params.T_ambient    = 30;

% -------------------------------------------------------------------------
% 2. LQR GAIN SCHEDULING (same Q/R as DLC / MUS — yaw-dominant)
% -------------------------------------------------------------------------
disp('-> Synthesizing LQR Feedback Gains...');
Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]);
R_lqr = 1/(2500^2);
V_op  = 5:5:85;
K_scheduled = zeros(length(V_op), 2);

for i = 1:length(V_op)
    vx_t = V_op(i);
    A_lin = [ -(veh_params.ks_f + veh_params.ks_r) / (veh_params.m * vx_t), ...
              ((veh_params.ks_r * veh_params.lr - veh_params.ks_f * veh_params.lf) / (veh_params.m * vx_t)) - vx_t;
              (veh_params.ks_r * veh_params.lr - veh_params.ks_f * veh_params.lf) / (veh_params.Iz * vx_t), ...
              -(veh_params.ks_f * veh_params.lf^2 + veh_params.ks_r * veh_params.lr^2) / (veh_params.Iz * vx_t) ];
    B_lin = [ veh_params.ks_f / veh_params.m,              0;
              (veh_params.lf * veh_params.ks_f) / veh_params.Iz, 1/veh_params.Iz ];
    try
        K_scheduled(i, :) = lqr(A_lin, B_lin(:,2), Q_lqr, R_lqr);
    catch
        if i > 1, K_scheduled(i, :) = K_scheduled(i-1, :); end
    end
end

% -------------------------------------------------------------------------
% 3. SCENARIO DEFINITION: 150 m RADIUS CORNER + TRAIL BRAKING
% -------------------------------------------------------------------------
disp('-> Configuring Cornering-Under-Braking scenario...');

sim_params.dt    = 0.001;
sim_params.t_end = 35.0;
t_sim = (0 : sim_params.dt : sim_params.t_end)';

%--- Target speed: 120 km/h hold, then brake to 60 km/h ---
V_corner_ms  = 120.0 / 3.6;   % 33.33 m/s steady-state
V_exit_ms    = 60.0  / 3.6;   % 16.67 m/s after braking
t_settle     = 12.0;           % time to reach steady circular state
t_brake      = t_settle;       % braking starts here
t_release    = t_brake + 7.0;  % braking released here

vx_target = V_corner_ms * ones(size(t_sim));
for i = 1:length(t_sim)
    if t_sim(i) >= t_brake && t_sim(i) < t_release
        % Linear ramp down from V_corner to V_exit over 2 s, then hold
        ramp_dur = 2.0;
        if t_sim(i) < t_brake + ramp_dur
            frac = (t_sim(i) - t_brake) / ramp_dur;
            vx_target(i) = V_corner_ms - (V_corner_ms - V_exit_ms) * frac;
        else
            vx_target(i) = V_exit_ms;
        end
    elseif t_sim(i) >= t_release
        vx_target(i) = V_corner_ms;   % re-accelerate after brake release
    end
end
vx_target_array = [t_sim, vx_target];

%--- Steering: constant road-wheel angle for 150 m radius ---
% Ackermann + understeer correction:
%   delta = (L/R) * (1 + Kus * V^2)
R_target  = 150.0;           % m
K_us      = 0.0015;          % understeeer gradient [s^2/m^2]
delta_road_rad = (veh_params.L / R_target) * (1 + K_us * V_corner_ms^2);
% delta_cmd is the steering-WHEEL input; Steer_ff divides by 3 for road wheel
delta_sw_rad = delta_road_rad * 3.0;    % steering-wheel equivalent

steer_rad     = zeros(size(t_sim));
for i = 1:length(t_sim)
    if t_sim(i) >= 1.0    % apply steering 1 s after start
        steer_rad(i) = delta_sw_rad;
    end
end
steer_profile = [t_sim, steer_rad];

disp(sprintf('-> Target corner: R = %.0f m  |  V = %.1f km/h  |  Ay = %.3f G', ...
    R_target, V_corner_ms*3.6, V_corner_ms^2 / (R_target * veh_params.g)));

%--- Load transfer prediction (for documentation / expected FC limits) ---
Ay_ss   = V_corner_ms^2 / R_target;   % lateral accel [m/s^2]
dFz_lat = veh_params.m * Ay_ss * veh_params.h_cg / veh_params.tw;  % total lateral transfer [N]
dFz_rl  = dFz_lat * veh_params.roll_bias_r;
Fz_rl_est = veh_params.m * veh_params.g / 4 - dFz_rl;
Fz_rr_est = veh_params.m * veh_params.g / 4 + dFz_rl;
Fy_rear   = veh_params.m * Ay_ss * veh_params.lr / veh_params.L;
Fy_rl_est = Fy_rear * Fz_rl_est / (Fz_rl_est + Fz_rr_est);
Fy_rr_est = Fy_rear * Fz_rr_est / (Fz_rl_est + Fz_rr_est);
FC_rl_est = sqrt(max((veh_params.mu_max * Fz_rl_est)^2 - Fy_rl_est^2, 0)) * 0.88;
FC_rr_est = sqrt(max((veh_params.mu_max * Fz_rr_est)^2 - Fy_rr_est^2, 0)) * 0.88;
FC_rl_old = veh_params.mu_max * Fz_rl_est * 0.90;    % existing allocator limit
FC_rr_old = veh_params.mu_max * Fz_rr_est * 0.90;

disp(sprintf('-> Predicted steady-state Fz_rl = %.0f N  |  Fz_rr = %.0f N', Fz_rl_est, Fz_rr_est));
disp(sprintf('-> Predicted Fy_rl = %.0f N  |  Fy_rr = %.0f N  (rear axle)', Fy_rl_est, Fy_rr_est));
disp(sprintf('-> QP   Fx_avail_rl = %.0f N  |  Fx_avail_rr = %.0f N', FC_rl_est, FC_rr_est));
disp(sprintf('-> OLD  Fx_avail_rl = %.0f N  |  Fx_avail_rr = %.0f N', FC_rl_old, FC_rr_old));
disp(sprintf('-> Inner wheel OVER-ESTIMATED by OLD allocator: %.1f%%', ...
    (FC_rl_old - FC_rl_est) / FC_rl_est * 100));

%--- Friction profile: uniform dry tarmac throughout ---
mu_high         = 1.2;
mu_left         = mu_high * ones(size(t_sim));
mu_right        = mu_high * ones(size(t_sim));
mu_profile_left  = [t_sim, mu_left];
mu_profile_right = [t_sim, mu_right];

%--- Dummy straight track to satisfy Scopes/pure-pursuit geometry blocks ---
X_track = [0; 5000];
Y_track = [0; 0];
assignin('base', 'X_track', X_track);
assignin('base', 'Y_track', Y_track);

%--- Initial conditions ---
sim_params.initial_speed = 0.1;
sim_params.initial_X     = 0;
sim_params.initial_Y     = 0;
sim_params.initial_psi   = 0;

% Expose key event times for post-processing scripts
t_brake_start   = t_brake;
t_brake_release = t_release;

disp('--- CUB Setup Complete ---');
disp(sprintf('   Braking event:  t = %.1f s  to  t = %.1f s', t_brake_start, t_brake_release));
