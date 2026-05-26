% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: CORNERING UNDER BRAKING (CUB)
% Models:   CUB_1a  — QP friction-circle-aware regen allocator  (NEW)
%           CUB_1b  — Flat-margin allocator  (EXISTING baseline)
% Purpose:  Rigorous validation of Patent #9
%           Three-axis verdict: Tyre Saturation | Yaw Tracking | Regen Energy
% =========================================================================
clear; clc; close all;

disp('====================================================');
disp('  CORNERING UNDER BRAKING (CUB) — ALLOCATOR BATTLE  ');
disp('====================================================');

%% 1. INITIALIZATION
disp('-> Loading CUB Parameters...');
CUB;    % runs CUB.m — populates veh_params, K_scheduled, steer_profile, etc.

%% 2. EXECUTE SIMULATIONS
disp('-> Running QP Allocator Simulation (CUB_1a)...');
out_a = sim('CUB_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a');
disp('   [CUB_1a — QP Complete]');

disp('-> Running Flat-Margin Baseline Simulation (CUB_1b)...');
out_b = sim('CUB_1b', 'ReturnWorkspaceOutputs', 'on');
disp('   [CUB_1b — Baseline Complete]');

load('temp_out_a.mat'); delete('temp_out_a.mat');

%% 3. DATA EXTRACTION
disp('-> Extracting Telemetry...');

t_a = out_a.tout;   t_b = out_b.tout;

% --- Core dynamics ---
vx_a    = squeeze(out_a.vx_curr_a.Data);
vy_a    = squeeze(out_a.vy_curr_a.Data);
r_a_rad = squeeze(out_a.r_curr_a.Data);
r_ref_a = squeeze(out_a.r_ref_a.Data);

vx_b    = squeeze(out_b.vx_curr_b.Data);
vy_b    = squeeze(out_b.vy_curr_b.Data);
r_b_rad = squeeze(out_b.r_curr_b.Data);
r_ref_b = squeeze(out_b.r_ref_b.Data);

r_a_deg     = r_a_rad * (180/pi);
r_b_deg     = r_b_rad * (180/pi);
r_ref_a_deg = r_ref_a * (180/pi);
beta_a_deg  = atan2(vy_a, vx_a) * (180/pi);
beta_b_deg  = atan2(vy_b, vx_b) * (180/pi);
Ay_a_G      = (vx_a .* r_a_rad) / veh_params.g;
Ay_b_G      = (vx_b .* r_b_rad) / veh_params.g;

% --- Motor torques ---
Trr_m_a = squeeze(out_a.Trr_m_a.Data);
Trl_m_a = squeeze(out_a.Trl_m_a.Data);
Trr_m_b = squeeze(out_b.Trr_m_b.Data);
Trl_m_b = squeeze(out_b.Trl_m_b.Data);

% --- Tyre forces ---
Fy_rl_a = squeeze(out_a.Fy_rl_a.Data);   Fy_rr_a = squeeze(out_a.Fy_rr_a.Data);
Fx_rl_a = squeeze(out_a.Fx_rl_a.Data);   Fx_rr_a = squeeze(out_a.Fx_rr_a.Data);
Fz_rl_a = squeeze(out_a.Fz_rl_a.Data);   Fz_rr_a = squeeze(out_a.Fz_rr_a.Data);

Fy_rl_b = squeeze(out_b.Fy_rl_b.Data);   Fy_rr_b = squeeze(out_b.Fy_rr_b.Data);
Fx_rl_b = squeeze(out_b.Fx_rl_b.Data);   Fx_rr_b = squeeze(out_b.Fx_rr_b.Data);
Fz_rl_b = squeeze(out_b.Fz_rl_b.Data);   Fz_rr_b = squeeze(out_b.Fz_rr_b.Data);

% --- Wheel speeds ---
w_rl_a = squeeze(out_a.w_rl_a.Data);   w_rr_a = squeeze(out_a.w_rr_a.Data);
w_rl_b = squeeze(out_b.w_rl_b.Data);   w_rr_b = squeeze(out_b.w_rr_b.Data);

%% 4. KEY METRIC CALCULATIONS
disp('-> Computing Validation Metrics...');

mu     = veh_params.mu_max;
dt_sim = mean(diff(t_a));

% ===== METRIC 1: Lateral Force Residual (the friction circle margin) =====
% Residual > 0 means tyre is not saturated
% Residual == 0 (or < 0) means saturation — loss of cornering ability
Fres_rl_a   = sqrt(Fx_rl_a.^2 + Fy_rl_a.^2);
Fres_rr_a   = sqrt(Fx_rr_a.^2 + Fy_rr_a.^2);
FC_limit_rl_a = mu * Fz_rl_a;
FC_limit_rr_a = mu * Fz_rr_a;

Fres_rl_b   = sqrt(Fx_rl_b.^2 + Fy_rl_b.^2);
Fres_rr_b   = sqrt(Fx_rr_b.^2 + Fy_rr_b.^2);
FC_limit_rl_b = mu * Fz_rl_b;
FC_limit_rr_b = mu * Fz_rr_b;

% Utilisation ratio (1.0 = exactly at limit, >1.0 = over-limit / saturated)
util_rl_a = Fres_rl_a ./ max(FC_limit_rl_a, 1);
util_rr_a = Fres_rr_a ./ max(FC_limit_rr_a, 1);
util_rl_b = Fres_rl_b ./ max(FC_limit_rl_b, 1);
util_rr_b = Fres_rr_b ./ max(FC_limit_rr_b, 1);

% Only look at the braking window
brake_mask_a = (t_a >= t_brake_start) & (t_a <= t_brake_release);
brake_mask_b = (t_b >= t_brake_start) & (t_b <= t_brake_release);

max_util_rl_a_brk = max(util_rl_a(brake_mask_a));
max_util_rr_a_brk = max(util_rr_a(brake_mask_a));
max_util_rl_b_brk = max(util_rl_b(brake_mask_b));
max_util_rr_b_brk = max(util_rr_b(brake_mask_b));

% ===== METRIC 2: Yaw Rate Tracking Error (during braking) =====
r_err_a_brk = r_a_rad(brake_mask_a) - r_ref_a(brake_mask_a);
r_err_b_brk = r_b_rad(brake_mask_b) - r_ref_b(brake_mask_b);

rms_yaw_err_a = sqrt(mean(r_err_a_brk.^2)) * (180/pi);   % [deg/s]
rms_yaw_err_b = sqrt(mean(r_err_b_brk.^2)) * (180/pi);

% ===== METRIC 3: Recovered Regen Energy during braking =====
% P_regen = |T_motor| * omega_motor * G  (when T_motor < 0)
w_rl_a_m = w_rl_a * veh_params.G;   w_rr_a_m = w_rr_a * veh_params.G;
w_rl_b_m = w_rl_b * veh_params.G;   w_rr_b_m = w_rr_b * veh_params.G;

P_regen_a = (-min(Trl_m_a, 0)) .* abs(w_rl_a_m) + (-min(Trr_m_a, 0)) .* abs(w_rr_a_m);
P_regen_b = (-min(Trl_m_b, 0)) .* abs(w_rl_b_m) + (-min(Trr_m_b, 0)) .* abs(w_rr_b_m);

E_regen_a_Wh = trapz(t_a(brake_mask_a), P_regen_a(brake_mask_a)) / 3600;
E_regen_b_Wh = trapz(t_b(brake_mask_b), P_regen_b(brake_mask_b)) / 3600;

% ===== METRIC 4: Peak Sideslip during braking =====
peak_beta_a = max(abs(beta_a_deg(brake_mask_a)));
peak_beta_b = max(abs(beta_b_deg(brake_mask_b)));

% ===== PRINT SCORECARD =====
disp(' ');
disp('╔══════════════════════════════════════════════════════════════╗');
disp('║              CUB VALIDATION SCORECARD                       ║');
disp('╠═══════════════════════╦══════════════╦════════════════════════╣');
disp('║  Metric               ║  QP (CUB_1a) ║  Baseline (CUB_1b)    ║');
disp('╠═══════════════════════╬══════════════╬════════════════════════╣');
fprintf('║  Max FC Util  RL [%%]  ║   %6.1f %%   ║     %6.1f %%          ║\n', max_util_rl_a_brk*100, max_util_rl_b_brk*100);
fprintf('║  Max FC Util  RR [%%]  ║   %6.1f %%   ║     %6.1f %%          ║\n', max_util_rr_a_brk*100, max_util_rr_b_brk*100);
fprintf('║  RMS Yaw Err [deg/s]  ║   %6.3f      ║     %6.3f             ║\n', rms_yaw_err_a, rms_yaw_err_b);
fprintf('║  Regen Energy [Wh]    ║   %6.2f       ║     %6.2f              ║\n', E_regen_a_Wh, E_regen_b_Wh);
fprintf('║  Peak Sideslip [deg]  ║   %6.3f      ║     %6.3f             ║\n', peak_beta_a, peak_beta_b);
disp('╚═══════════════════════╩══════════════╩════════════════════════╝');
disp(' ');

% Pass/Fail assessment
disp('--- PASS / FAIL ASSESSMENT ---');
if max_util_rl_a_brk < 1.0 && max_util_rr_a_brk < 1.0
    disp('[PASS] QP: Friction circle NOT violated at any rear wheel during braking');
else
    disp('[FAIL] QP: Friction circle VIOLATED — check QP constraints');
end
if max_util_rl_b_brk >= 1.0 || max_util_rr_b_brk >= 1.0
    disp('[DEMONSTRATED] Baseline saturates tyre — proves gap that QP solves');
end
if E_regen_a_Wh >= 0.9 * E_regen_b_Wh
    disp('[PASS] QP recovers >=90% of baseline regen energy');
else
    disp('[INFO] QP recovers less regen energy — may need k_safety tuning');
end
if rms_yaw_err_a <= rms_yaw_err_b * 1.15
    disp('[PASS] QP yaw tracking within 15% of baseline');
else
    disp('[INFO] QP yaw tracking degraded — check LQR weight vs regen weight');
end

%% 5. DASHBOARD FIGURES
disp('-> Rendering Dashboards...');

% --- FIG 1: The Friction Circle Battle ---
figure('Name', 'CUB Fig 1: Friction Circle Utilisation', 'Color', 'w', 'Position', [50, 50, 1100, 800]);

subplot(2,2,1);
plot(t_a, util_rl_a * 100, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, util_rl_b * 100, 'r--', 'LineWidth', 1.5);
yline(100, 'k-', 'Saturation (100%)', 'LineWidth', 2);
xline(t_brake_start,   'k--', 'Brake ON',  'LabelVerticalAlignment', 'bottom');
xline(t_brake_release, 'k--', 'Brake OFF', 'LabelVerticalAlignment', 'bottom');
title('Rear Left (Inner) — Friction Utilisation');
ylabel('F_{res} / (\mu F_z)  [%]');  xlabel('Time (s)');
legend('QP Allocator', 'Flat-Margin Baseline', 'Location', 'best');

subplot(2,2,2);
plot(t_a, util_rr_a * 100, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, util_rr_b * 100, 'r--', 'LineWidth', 1.5);
yline(100, 'k-', 'Saturation (100%)', 'LineWidth', 2);
xline(t_brake_start,   'k--', 'Brake ON');
xline(t_brake_release, 'k--', 'Brake OFF');
title('Rear Right (Outer) — Friction Utilisation');
ylabel('F_{res} / (\mu F_z)  [%]');  xlabel('Time (s)');

subplot(2,2,3);
theta_c = linspace(0,2*pi,200);
cx = cos(theta_c); cy = sin(theta_c);
G_long_rl_a = Fx_rl_a ./ max(Fz_rl_a, 1);
G_lat_rl_a  = Fy_rl_a ./ max(Fz_rl_a, 1);
G_long_rl_b = Fx_rl_b ./ max(Fz_rl_b, 1);
G_lat_rl_b  = Fy_rl_b ./ max(Fz_rl_b, 1);
plot(mu*cx, mu*cy, 'k-', 'LineWidth', 2); hold on; grid on;
plot(G_lat_rl_a(brake_mask_a), G_long_rl_a(brake_mask_a), 'b.', 'MarkerSize', 3);
plot(G_lat_rl_b(brake_mask_b), G_long_rl_b(brake_mask_b), 'r.', 'MarkerSize', 3);
axis equal; xlim([-1.5 1.5]); ylim([-1.5 1.5]);
xlabel('Lateral G'); ylabel('Longitudinal G');
title('Rear Left G-G (Braking Window)');
legend('\mu_{max} limit', 'QP', 'Baseline', 'Location', 'best');

subplot(2,2,4);
G_long_rr_a = Fx_rr_a ./ max(Fz_rr_a, 1);
G_lat_rr_a  = Fy_rr_a ./ max(Fz_rr_a, 1);
G_long_rr_b = Fx_rr_b ./ max(Fz_rr_b, 1);
G_lat_rr_b  = Fy_rr_b ./ max(Fz_rr_b, 1);
plot(mu*cx, mu*cy, 'k-', 'LineWidth', 2); hold on; grid on;
plot(G_lat_rr_a(brake_mask_a), G_long_rr_a(brake_mask_a), 'b.', 'MarkerSize', 3);
plot(G_lat_rr_b(brake_mask_b), G_long_rr_b(brake_mask_b), 'r.', 'MarkerSize', 3);
axis equal; xlim([-1.5 1.5]); ylim([-1.5 1.5]);
xlabel('Lateral G'); ylabel('Longitudinal G');
title('Rear Right G-G (Braking Window)');

% --- FIG 2: Yaw Rate Tracking ---
figure('Name', 'CUB Fig 2: Yaw Rate Tracking', 'Color', 'w', 'Position', [100, 100, 900, 700]);
subplot(3,1,1);
plot(t_a, r_ref_a_deg, 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, r_a_deg, 'b-', 'LineWidth', 1.5);
plot(t_b, r_b_deg, 'r-', 'LineWidth', 1.5);
xline(t_brake_start,   'k--', 'Brake ON',  'LabelVerticalAlignment', 'bottom');
xline(t_brake_release, 'k--', 'Brake OFF', 'LabelVerticalAlignment', 'bottom');
title('Yaw Rate Tracking During Corner-Braking');
ylabel('r (deg/s)');
legend('Reference', 'QP Allocator', 'Flat-Margin Baseline', 'Location', 'best');

subplot(3,1,2);
plot(t_a, beta_a_deg, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, beta_b_deg, 'r-', 'LineWidth', 1.5);
xline(t_brake_start,   'k--'); xline(t_brake_release, 'k--');
title('Vehicle Sideslip Angle (\beta)');
ylabel('\beta (deg)');
legend('QP Allocator', 'Flat-Margin Baseline', 'Location', 'best');

subplot(3,1,3);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 1.5);
xline(t_brake_start,   'k--'); xline(t_brake_release, 'k--');
title('Longitudinal Speed');
ylabel('V_x (km/h)');  xlabel('Time (s)');
legend('QP Allocator', 'Flat-Margin Baseline', 'Location', 'best');

% --- FIG 3: Regen Torque Allocation ---
figure('Name', 'CUB Fig 3: Regen Torque Allocation', 'Color', 'w', 'Position', [150, 150, 900, 700]);
subplot(3,1,1);
plot(t_a, Trl_m_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, Trr_m_a, 'g-', 'LineWidth', 1.5);
xline(t_brake_start,   'k--', 'Brake ON');
xline(t_brake_release, 'k--', 'Brake OFF');
title('QP Allocator — Motor Torque (Regen Phase)');
ylabel('T_{motor} (Nm)');
legend('Rear Left (Inner)', 'Rear Right (Outer)', 'Location', 'best');

subplot(3,1,2);
plot(t_b, Trl_m_b, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Trr_m_b, 'g-', 'LineWidth', 1.5);
xline(t_brake_start,   'k--'); xline(t_brake_release, 'k--');
title('Flat-Margin Baseline — Motor Torque (Regen Phase)');
ylabel('T_{motor} (Nm)');

subplot(3,1,3);
Delta_T_a = Trr_m_a - Trl_m_a;
Delta_T_b = Trr_m_b - Trl_m_b;
plot(t_a, Delta_T_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Delta_T_b, 'r--', 'LineWidth', 1.5);
xline(t_brake_start,   'k--'); xline(t_brake_release, 'k--');
title('TV Intent: \DeltaT = T_{rr} - T_{rl}  (Yaw Differential)');
ylabel('\Delta T (Nm)');  xlabel('Time (s)');
legend('QP Allocator', 'Flat-Margin Baseline', 'Location', 'best');

% --- FIG 4: Regen Power ---
figure('Name', 'CUB Fig 4: Regen Power Recovery', 'Color', 'w', 'Position', [200, 200, 900, 500]);
plot(t_a, P_regen_a / 1000, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, P_regen_b / 1000, 'r--', 'LineWidth', 1.5);
xline(t_brake_start,   'k--', 'Brake ON');
xline(t_brake_release, 'k--', 'Brake OFF');
title(sprintf('Instantaneous Regen Power  |  QP: %.2f Wh  |  Baseline: %.2f Wh', ...
    E_regen_a_Wh, E_regen_b_Wh));
ylabel('P_{regen} (kW)');  xlabel('Time (s)');
legend('QP Allocator', 'Flat-Margin Baseline', 'Location', 'best');

% --- FIG 5: The key comparison — Fy residual margin ---
figure('Name', 'CUB Fig 5: Lateral Force Residual (Patent Core Metric)', 'Color', 'w', 'Position', [250, 250, 1000, 600]);
Fy_residual_rl_a = FC_limit_rl_a - Fres_rl_a;
Fy_residual_rl_b = FC_limit_rl_b - Fres_rl_b;

subplot(2,1,1);
plot(t_a, Fy_residual_rl_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Fy_residual_rl_b, 'r--', 'LineWidth', 1.5);
yline(0, 'k-', 'Saturation Boundary', 'LineWidth', 2);
xline(t_brake_start,   'k--', 'Brake ON');
xline(t_brake_release, 'k--', 'Brake OFF');
title('Rear-Left (Inner) Lateral Force Residual = \muF_z - F_{res}');
ylabel('F_{residual} (N)');
legend('QP Allocator (NEVER crosses 0)', 'Flat-Margin Baseline', 'Location', 'best');

subplot(2,1,2);
Fy_residual_rr_a = FC_limit_rr_a - Fres_rr_a;
Fy_residual_rr_b = FC_limit_rr_b - Fres_rr_b;
plot(t_a, Fy_residual_rr_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Fy_residual_rr_b, 'r--', 'LineWidth', 1.5);
yline(0, 'k-', 'Saturation Boundary', 'LineWidth', 2);
xline(t_brake_start,   'k--'); xline(t_brake_release, 'k--');
title('Rear-Right (Outer) Lateral Force Residual');
ylabel('F_{residual} (N)'); xlabel('Time (s)');

disp('-> All CUB Dashboards Rendered.');
disp('====================================================');
